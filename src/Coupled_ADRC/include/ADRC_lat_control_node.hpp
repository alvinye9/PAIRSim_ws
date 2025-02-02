/* Copyright 2021 Will Bryan

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

//#ifndef KIN_CONTROL_HPP
//#define KIN_CONTROL_HPP

#include <math.h>
#include <unistd.h>
#include <cmath>

#include <chrono>
#include <cstdlib>
#include <sstream>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include "blackandgold_msgs/msg/error_report.hpp"
#include "blackandgold_msgs/msg/speed_reference_request.hpp"
#include "blackandgold_msgs/msg/adrc.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace control {

class ADRCLatControl : public rclcpp::Node {
 public:
  ADRCLatControl() : Node("ADRCLatControlNode") {
    // setup QOS to be best effort
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

    // Extended-state observer (ESO): 
    /*Aobs << 0.6671, 0.0, -9.1118e-15, -5.6245e-17, // Aobs = A-L*C Discrete
    0.0, 0.6703, 0.0, 0.0,
    -5.9339e-12, 0.0, 0.4296, 0.0069,
    -2.2233e-10, 0.0, -9.6633, 0.9450;


    Bobs <<  3.6803, 0.1757, 0.0,  -0.4428,
            0.0, 0.0, 0.3297, 0.0082,
            3.1539, -0.0150, 0.0, 0.4071,
            -25.2236, 0.1203, 0.0, 10.9696; */
    //obs_in_prev << 0,0;

    // Parameters
    this->declare_parameter("min_lookahead", 4.0);
    this->declare_parameter("max_lookahead", 50.0);
    this->declare_parameter("lookahead_speed_ratio", 0.75);
    this->declare_parameter("proportional_gain", 0.2);
    this->declare_parameter("derivative_gain", 0.2);
    this->declare_parameter("vehicle.wheelbase", 2.97);
    this->declare_parameter("max_steer_angle", 17.5);  // 15 deg * 2 because ratio is wrong?? 
    this->declare_parameter("steering_override_threshold", 4.0);
    this->declare_parameter("max_lat_error", 4.0);
    this->declare_parameter("max_lookahead_error", 8.0);
    this->declare_parameter("auto_enabled", false);
    this->declare_parameter("mute", true);
    this->declare_parameter("debug", true);
    this->declare_parameter("K1", 0.0);
    this->declare_parameter("K2", 0.0);
    this->declare_parameter("vehicle.cg_to_front", 1.0);
    this->declare_parameter("vehicle.cg_to_rear", 1.0);
    this->declare_parameter("vehicle.tire_stiffness_front", 0.0);
    this->declare_parameter("vehicle.tire_stiffness_rear", 0.0);
    this->declare_parameter("vehicle.mass", 0.0);
    this->declare_parameter("vehicle.inertia", 0.0);
    this->declare_parameter("max_one_step_lat_err_rate", 2000.0);
    this->declare_parameter("max_avg_lat_err_rate", 500.0);
    this->declare_parameter("lat_err_history_length", 4);

    this->declare_parameter("curv_to_velocity_constant", 0.5);
    this->declare_parameter("stopping_distance_multiplier", 1.0);
    this->declare_parameter("max_possible_deceleration", 2.0); // m/s/s should be same as graceful_stop_acceleration in speed_reference_generator

    K1 = this->get_parameter("K1").as_double();
    K2 = this->get_parameter("K2").as_double();
    lf = this->get_parameter("vehicle.cg_to_front").as_double();
    lr = this->get_parameter("vehicle.cg_to_rear").as_double();
    Caf = this->get_parameter("vehicle.tire_stiffness_front").as_double();
    Car = this->get_parameter("vehicle.tire_stiffness_rear").as_double();
    m = this->get_parameter("vehicle.mass").as_double();
    Izz = this->get_parameter("vehicle.inertia").as_double();

    min_la = this->get_parameter("min_lookahead").as_double();
    max_la = this->get_parameter("max_lookahead").as_double();
    la_ratio = this->get_parameter("lookahead_speed_ratio").as_double();
    max_steer_angle = this->get_parameter("max_steer_angle").as_double();
    pub_debug_signals = this->get_parameter("debug").as_bool();

    // Publishers
    pubSteeringCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd", 1);
    pubLookaheadError_ = this->create_publisher<std_msgs::msg::Float64>("lookahead_error", 10);
    pubLatError_ = this->create_publisher<std_msgs::msg::Float64>("lateral_error", 10);
    pubErrorReporter_ =
        this->create_publisher<blackandgold_msgs::msg::ErrorReport>("/vehicle/errors", 5);
    controlDebugSignals_ = this->create_publisher<blackandgold_msgs::msg::ADRC>("/control/ADRC_debug_signals", 10);
    pubYawRef = this->create_publisher<std_msgs::msg::Float32>("/control/desired_heading", 10);
    pubCurvature = this->create_publisher<std_msgs::msg::Float32>("/control/curvature", 10);
    pubCurvDesiredVelocity_ = this->create_publisher<blackandgold_msgs::msg::SpeedReferenceRequest>("/control/curvature_max_velocity", 1);
    
    sleep(3);

    // Subscribers
    subPath_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/offset_path", 1,
        std::bind(&ADRCLatControl::receivePath, this, std::placeholders::_1));
    subVelocity_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", qos,
        std::bind(&ADRCLatControl::receiveVelocity, this, std::placeholders::_1));
    subPathCurvature_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/planning/front_path/curvature", qos, std::bind(&ADRCLatControl::calculateCurvatureVelocity, this, std::placeholders::_1));
        //"/planning/front_path/curvature", qos, std::bind(&ADRCLatControl::receiveCurvature, this, std::placeholders::_1));
    subFilteredOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/global_filtered", qos, std::bind(&ADRCLatControl::receiveOdom, this, std::placeholders::_1)); 

    // Create Timer
    //rclcpp::Clock ros_clock(RCL_ROS_TIME);
    control_timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(static_cast<int>(dt * 1000)), 
                                          std::bind(&ADRCLatControl::controlCallback, this));


    //control_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
    //                                        std::bind(&ADRCLatControl::controlCallback, this));

    lookahead_error.data = 0.0;
    lat_error.data = 0.0;
    steering_cmd.data = 0.0;

    this->max_lat_error = this->get_parameter("max_lat_error").as_double();
    this->max_lookahead_error = this->get_parameter("max_lookahead_error").as_double();

  };

  std_msgs::msg::Float32 steering_cmd;
  std_msgs::msg::Float64 lookahead_error;
  std_msgs::msg::Float64 lat_error;
  std_msgs::msg::Float32MultiArray path_curvatures;
  blackandgold_msgs::msg::ErrorReport error_msg;


 private:
  struct Quaternion {
    double w, x, y, z;
  };

  struct EulerAngle {
    double pitch, roll, yaw;
  };

  void controlCallback();
  void calculateFFW();
  void calculateFB(double dt);
  void calculateSteeringCmd();
  void setCmdsToZeros();
  void publishSteering();
  void publishDebugSignals();
  void receivePath(const nav_msgs::msg::Path::SharedPtr msg);
  void receiveCurvature(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  std::tuple<int, double> findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath,
                                             double desLookaheadValue);
  void receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
  void receiveOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  double quaternionToHeading(const Quaternion& q);
  void calculateCurvatureVelocity(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  float _dist(float x, float y, float z);

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubSteeringCmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLookaheadError_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLatError_;
  rclcpp::Publisher<blackandgold_msgs::msg::ErrorReport>::SharedPtr pubErrorReporter_;
  rclcpp::Publisher<blackandgold_msgs::msg::ADRC>::SharedPtr controlDebugSignals_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubYawRef;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubCurvature;
  rclcpp::Publisher<blackandgold_msgs::msg::SpeedReferenceRequest>::SharedPtr pubCurvDesiredVelocity_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr subVelocity_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subPathCurvature_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subFilteredOdom_;
  rclcpp::Time recv_time_;

  double curvature_ = 0.0;
  double speed_ = 0.0;
  double feedforward_ = 0.0;
  double feedback_ = 0.0;
  double steering_override_ = 0.0;
  double max_lat_error = 0.0;
  double max_lookahead_error = 0.0;
  double dt = 0.01;
  double previous_lat_error = 0.0;
  double lat_error_derivative = 0.0;
  double desired_yaw_rate = 0.0;
  nav_msgs::msg::Odometry odom;
  double previous_yaw_rate_error = 0.0;
  double previous_lookahead_error = 0.0;
  int idx = 0;
  double yaw_error_ = 0.0;
  double previous_derivative = 0.0;
  double heading = 0.0;
  double previous_yaw_rate = 0.0;
  double angular_acceleration = 0.0;
  double control_signal = 0.0;
  double yaw_ref = 0.0;
  double xi_prev = 0.0;
  double yaw_rate = 0.0;
  double lat_speed = 0.0;

  double a22 = 0.0;
  double a24 = 0.0;
  double a42 = 0.0;
  double a44 = 0.0;

  double K1;
  double K2;
  double lf;
  double lr;
  double Caf;
  double Car;
  double m;
  double Izz;
  double min_la;
  double max_la;
  double la_ratio; 
  double max_steer_angle;
  bool pub_debug_signals;


  nav_msgs::msg::Path::SharedPtr path_msg;
  blackandgold_msgs::msg::SpeedReferenceRequest max_curv_velocity_message;

  // Extended-state observer variables:
  double previous_yaw = 0.0;
  double previous_control_signal = 0.0;
  double previous_lat_speed = 0.0;

  Eigen::Matrix<double, 4, 1> x_hat; // Initialize vector of estimated states 
  Eigen::Matrix<double, 4, 1> x_hat_prev; // Initialize vector of estimated states 
  double xi_hat;
  double yaw_hat;
  double lat_speed_hat;
  double yaw_rate_hat;

  Eigen::Matrix<double, 4, 1> obs_in_prev; // Initialize vector of observer inputs [u; y]

  // apply moving average filter to look ahead history
  double look_ahead_error_history[4] = {0.0};
  int lae_history_counter = 0;
  double lae_history_avg = 0.;

 // history for lateral error safety checks
  int lat_err_history_counter = 0;
  std::vector<double> lat_err_history;
  std::vector<rclcpp::Time> lat_err_time_history; 

  // ESO Matrices:
  Eigen::Matrix<double, 4, 4> Aobs; 
  Eigen::Matrix<double, 4,4> Bobs;


};  // end of class

}  // namespace control

//#endif
