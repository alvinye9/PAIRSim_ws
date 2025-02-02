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
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include "deep_orange_msgs/msg/joystick_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "system_dynamics.hpp"


namespace control {

class CoupledADRC : public rclcpp::Node {
 public:
  CoupledADRC() : Node("CoupledADRCNode") {
    // setup QOS to be best effort
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

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
    this->declare_parameter("K1l", 0.0);
    this->declare_parameter("K1", 0.0);
    this->declare_parameter("K2", 0.0);
    this->declare_parameter("K3", 0.0);
    this->declare_parameter("K4", 0.0);
    this->declare_parameter("vehicle.cg_to_front", 1.0);
    this->declare_parameter("vehicle.cg_to_rear", 1.0);
    this->declare_parameter("vehicle.tire_stiffness_front", 0.0);
    this->declare_parameter("vehicle.tire_stiffness_rear", 0.0);
    this->declare_parameter("vehicle.mass", 0.0);
    this->declare_parameter("vehicle.inertia", 0.0);
    this->declare_parameter("max_one_step_lat_err_rate", 2000.0);
    this->declare_parameter("max_avg_lat_err_rate", 500.0);
    this->declare_parameter("lat_err_history_length", 4);
    this->declare_parameter("disturbance_model", 1);
    this->declare_parameter("curv_to_velocity_constant", 0.5);
    this->declare_parameter("stopping_distance_multiplier", 1.0);
    this->declare_parameter("max_possible_deceleration", 2.0); // m/s/s should be same as graceful_stop_acceleration in speed_reference_generator
    this->declare_parameter("cov_Q", 0.02);
    this->declare_parameter("cov_R", 0.1);

    this->declare_parameter("dummy_param", 0.0);

    K1 = this->get_parameter("K1").as_double();
    K2 = this->get_parameter("K2").as_double();
    K1l = this->get_parameter("K1l").as_double();

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
    disturbance_model = this->get_parameter("disturbance_model").as_int();
    //disturbance_model == 2 ? this->state_space_order = 7 : this->state_space_order = 6;

    // Publishers
    pubLookaheadError_ = this->create_publisher<std_msgs::msg::Float64>("lookahead_error", 10);
    pubLatError_ = this->create_publisher<std_msgs::msg::Float64>("lateral_error", 10);
    pubErrorReporter_ =
        this->create_publisher<blackandgold_msgs::msg::ErrorReport>("/vehicle/errors", 5);
    controlDebugSignals_ = this->create_publisher<blackandgold_msgs::msg::ADRC>("/control/ADRC_debug_signals", 10);
    pubLookaheadDistance_ = this->create_publisher<std_msgs::msg::Float32>("/lookahead_distance", 10);
    pubYawRef = this->create_publisher<std_msgs::msg::Float32>("/control/desired_heading", 10);
    pubCurvature = this->create_publisher<std_msgs::msg::Float32>("/control/curvature", 10);
    pubCurvDesiredVelocity_ = this->create_publisher<blackandgold_msgs::msg::SpeedReferenceRequest>("/control/curvature_max_velocity", 1);
    
    pubControlCmd_ = this->create_publisher<deep_orange_msgs::msg::JoystickCommand>("/control/adrc/accel_command", rclcpp::SensorDataQoS());
    pubSteeringCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd", 1);

    pubLookaheadMarker_ = this->create_publisher<visualization_msgs::msg::Marker>("/control/lookahead_distance_visz", 10);

    // pubLookaheadMarker = 

    //pubThrottleCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/accelerator_cmd", 1);
    //pubBrakeCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 1);
    //pubGearCmd_ = this->create_publisher<std_msgs::msg::Int8>("/joystick/gear_cmd", 1);

    pubDesiredAccel_ = this->create_publisher<std_msgs::msg::Float32>("/control/desired_acceleration", 1);

    sleep(3);

    RCLCPP_INFO(this->get_logger(), "Starting ADRC Coupled controller");

    // Subscribers
    subPath_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/front_path/offset_path", 1,
        std::bind(&CoupledADRC::receivePath, this, std::placeholders::_1));
    subVelocity_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", qos,
        std::bind(&CoupledADRC::receiveVelocity, this, std::placeholders::_1));
    subDesiredVelocity_ = this->create_subscription<std_msgs::msg::Float32>(
      "/planning/desired_velocity", qos,
      std::bind(&CoupledADRC::receiveDesiredVelocity, this, std::placeholders::_1));
    subPathCurvature_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/planning/front_path/curvature", qos, std::bind(&CoupledADRC::calculateCurvatureVelocity, this, std::placeholders::_1));
      //"/planning/front_path/curvature", qos, std::bind(&ADRCLatControl::receiveCurvature, this, std::placeholders::_1));
    subFilteredOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/global_filtered", qos, std::bind(&CoupledADRC::receiveOdom, this, std::placeholders::_1)); 

    subPtReport_ = this->create_subscription<deep_orange_msgs::msg::PtReport>(
      "/raptor_dbw_interface/pt_report", 1,
      std::bind(&CoupledADRC::receivePtReport, this, std::placeholders::_1));
    

    // Create Timer
    //rclcpp::Clock ros_clock(RCL_ROS_TIME);
    control_timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(static_cast<int>(dt * 1000)), 
                                          std::bind(&CoupledADRC::controlCallback, this));


    lookahead_error.data = 0.0;
    lat_error.data = 0.0;
    steering_cmd.data = 0.0;

    this->max_lat_error = this->get_parameter("max_lat_error").as_double();
    this->max_lookahead_error = this->get_parameter("max_lookahead_error").as_double();


    double steeringAngleGain = M_PI/180.*9.0/19.0;  // 1 degree steering angle = how many radians for wheel turn angle (delta in radians)
    // double brakingTorqueGain = 0.75*0.134*M_PI*0.008*0.008*4.;    // 1 command value = how much braking torque on the wheels
    double brakingTorqueGain = 2465.09319271;    // 1 command value = how much braking torque on the wheels
    double engineRPMUpperLimit = 6000.;
    double engineRPMLowerLimit = 1500.;
    // Engine Data - Hard Coded
    const double engineCurveRPM[17]={0., 1500., 3250., 3500., 3750., 4000., 4250., 4500., 4750., 5000., 
                                    5250., 5500., 5750., 6000., 6250., 6500., 6750.};
    // Output torque converted to N*m
    const double engineCurveTorque[17]={0.0, 108.47, 216.93, 322.68, 399.97, 481.32, 515.21, 508.43, 501.65, 490.81,
                                        485.38, 474.54, 463.69, 454.20, 436.57, 416.24, 393.19}; //FIX ME - first value edited! initially 0!
    static const uint8_t nGear = 6;
    // Gear ratio [including differential drive stage (*3)].
    const double gearRatio[nGear] = {8.7501, 5.625, 4.143, 3.345, 2.880, 2.667};

    const double torque_coeffs[11] = {-5.95520059e-06,  2.63075181e-04, -4.73910814e-03,  4.41159914e-02,
    -2.17594115e-01,  4.74959379e-01,  1.35258404e-01, -2.34751174e+00,
    3.27807309e+00, -6.94953662e-01,  2.65404324e-01};

    this->max_curv_velocity_message.origin = "ADRC_lat_control_node"; // FIXME: Change me
    this->max_curv_velocity_message.message = this->max_curv_velocity_message.origin;

  };

  deep_orange_msgs::msg::JoystickCommand control_cmd_msg;
  std_msgs::msg::Float32 steering_cmd;
  std_msgs::msg::Float64 lookahead_error;
  std_msgs::msg::Float64 lat_error;
  std_msgs::msg::Float32MultiArray path_curvatures;
  blackandgold_msgs::msg::ErrorReport error_msg;
  nav_msgs::msg::Path::SharedPtr path_msg;
  blackandgold_msgs::msg::SpeedReferenceRequest max_curv_velocity_message;


 private:
  struct Quaternion {
    double w, x, y, z;
  };

  struct EulerAngle {
    double pitch, roll, yaw;
  };

  vehiclePhysicsParamsType VP_;

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
  Eigen::Matrix<double, 6, 1> observe(void);
  double convertToBestGearThrottle();
  void receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg);
  void receiveDesiredVelocity(const std_msgs::msg::Float32::SharedPtr msg);
  void calculateThrottleBrakeFromTractionForce(double Ft);
  double interpolateEngineMaxTorque(double rpm);
  std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>> KalmanExtendedObserver(Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>);
  std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>> KalmanExtendedObserverRamp(Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>);


  rclcpp::TimerBase::SharedPtr control_timer_;

  // Actuators:
  rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr pubControlCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubSteeringCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmd_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubGearCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDesiredAccel_;
  // Other publishers:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLookaheadError_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLatError_;
  rclcpp::Publisher<blackandgold_msgs::msg::ErrorReport>::SharedPtr pubErrorReporter_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubLookaheadDistance_;
  rclcpp::Publisher<blackandgold_msgs::msg::ADRC>::SharedPtr controlDebugSignals_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubYawRef;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubCurvature;
  rclcpp::Publisher<blackandgold_msgs::msg::SpeedReferenceRequest>::SharedPtr pubCurvDesiredVelocity_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubLookaheadMarker_;

  // Subscribers:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subPathCurvature_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subFilteredOdom_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr subVelocity_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subDesiredVelocity_;
  rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr subPtReport_;


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
  double lookahead_distance;

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

  int current_gear_;
  double engine_speed_;
  double engine_running_;

  double K1;
  double K2;
  double K1l;
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
  double rear_left_wheel_speed;
  double rear_right_wheel_speed;
  double throttle_cmd;
  double brake_cmd;
  int gear_cmd = 1;
  const double RPM2RadPerSec = M_PI / 30.0;
  double desired_velocity;


  bool pub_debug_signals;

  
  // Extended-state observer variables:
  double previous_yaw = 0.0;
  double previous_control_signal = 0.0;
  double previous_lat_speed = 0.0;


  Eigen::Matrix<double, 7, 1> x_hat_prev; // Initialize vector of estimated states 
  Eigen::Matrix<double, 7, 7> Pk_prev; // Initialize covariance matrix of previous sample for Kalman Filter

  double steer;
  double F_traction;
  double xi_hat;
  double yaw_hat;
  double lat_speed_hat;
  double yaw_rate_hat;
  double speed_error;

  int disturbance_model = 1;

  Eigen::Matrix<double, 4, 1> obs_in_prev; // Initialize vector of observer inputs [u; y]
  //Eigen::Matrix<double, 7, 7> Pk;   

  // apply moving average filter to look ahead history
  double look_ahead_error_history[4] = {0.0};
  int lae_history_counter = 0;
  double lae_history_avg = 0.;

 // history for lateral error safety checks
  int lat_err_history_counter = 0;
  std::vector<double> lat_err_history;
  std::vector<rclcpp::Time> lat_err_time_history; 

};  // end of class

}  // namespace control

//#endif
