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

#ifndef KIN_CONTROL_HPP
#define KIN_CONTROL_HPP

#include <math.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "blackandgold_msgs/msg/error_report.hpp"
#include "blackandgold_msgs/msg/speed_reference_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

namespace control {

class KinControl : public rclcpp::Node {
 public:
  KinControl() : Node("KinControlNode") {
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
    this->declare_parameter("max_steer_angle", 30.0);  // 15 deg * 2 because ratio is wrong
    this->declare_parameter("max_lat_error", 4.0);
    this->declare_parameter("max_lookahead_error", 8.0);
    this->declare_parameter("auto_enabled", false);
    this->declare_parameter("mute", true);
    this->declare_parameter("curv_to_velocity_constant", 0.01);
    this->declare_parameter("max_possible_deceleration", 2.0); // m/s/s should be same as graceful_stop_acceleration in speed_reference_generator

    // Publishers
    pubSteeringCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd", 1);
    pubLookaheadError_ = this->create_publisher<std_msgs::msg::Float64>("lookahead_error", 10);
    pubLatError_ = this->create_publisher<std_msgs::msg::Float64>("lateral_error", 10);
    pubErrorReporter_ =
        this->create_publisher<blackandgold_msgs::msg::ErrorReport>("/vehicle/errors", 5);

    sleep(3);

    // Subscribers
    // TODO: use parameters to specify path topics
    subPath_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/front_path/offset_path", 1,
        std::bind(&KinControl::receivePath, this, std::placeholders::_1));

    subVelocity_ = this->create_subscription<std_msgs::msg::Float32>(
        "/localization/vehicle_speed", qos,
        std::bind(&KinControl::receiveVelocity, this, std::placeholders::_1));

    // Create Timer
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                             std::bind(&KinControl::controlCallback, this));

    lookahead_error.data = 0.0;
    lat_error.data = 0.0;
    steering_cmd.data = 0.0;

    this->max_lat_error = this->get_parameter("max_lat_error").as_double();
    this->max_lookahead_error = this->get_parameter("max_lookahead_error").as_double();

    this->max_curv_velocity_message.origin = "kin_control_curvature_limited_velocity";
    this->max_curv_velocity_message.message = "curvature limited velocity";
  };

  std_msgs::msg::Float32 steering_cmd;
  std_msgs::msg::Float64 lookahead_error;
  std_msgs::msg::Float64 lat_error;
  blackandgold_msgs::msg::SpeedReferenceRequest max_curv_velocity_message;
  blackandgold_msgs::msg::ErrorReport error_msg;

 private:
  void controlCallback();
  void calculateFFW();
  void calculateFB(double dt);
  void calculateSteeringCmd();
  void setCmdsToZeros();
  void publishSteering();
  void publishDebugSignals();
  void receivePath(const nav_msgs::msg::Path::SharedPtr msg);
  std::tuple<int, double> findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath,
                                             double desLookaheadValue);
  void receiveVelocity(const std_msgs::msg::Float32::SharedPtr msg);


  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubSteeringCmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLookaheadError_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLatError_;
  rclcpp::Publisher<blackandgold_msgs::msg::ErrorReport>::SharedPtr pubErrorReporter_;
  rclcpp::Publisher<blackandgold_msgs::msg::SpeedReferenceRequest>::SharedPtr pubCurvDesiredVelocity_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subVelocity_;
  rclcpp::Time recv_time_;

  nav_msgs::msg::Path::SharedPtr path_msg;

  double curvature_ = 0.0;
  double curvature_velocity = 0.0;  // The max velocity we may go based on how sharp of turn we are approaching
  double speed_ = 0.0;
  double feedforward_ = 0.0;
  double feedback_ = 0.0;
  double max_lat_error = 0.0;
  double max_lookahead_error = 0.0;
  double dt = 0.01;

  // apply moving average filter to look ahead history
  double look_ahead_error_history[4] = {0.0};
  int lae_history_counter = 0;
  double lae_history_avg = 0.;

};  // end of class

}  // namespace control

#endif
