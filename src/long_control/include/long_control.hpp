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

#ifndef LONG_CONTROL_HPP
#define LONG_CONTROL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"

#include "PID.hpp"

namespace control
{

class LongControl : public rclcpp::Node
{
  public:
    LongControl();

    std_msgs::msg::Float32 throttle_cmd;
    std_msgs::msg::Float32 brake_cmd;
    std_msgs::msg::Int8 gear_cmd;

  private:
    void controlCallback();
    void paramUpdateCallback();
    double calculateVelocityError();
    void calculateThrottleCmd(double vel_err);
    void calculateBrakeCmd(double vel_err);
    void setCmdsToZeros();
    void publishThrottleBrake();
    void shiftCallback();
    void receiveVelocity(const std_msgs::msg::Float32::SharedPtr msg);
    void receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg);
    void receiveDesiredVelocity(const std_msgs::msg::Float32::SharedPtr msg);
    void receiveSpinMon(const std_msgs::msg::Float32::SharedPtr msg);
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr param_timer_;
    rclcpp::TimerBase::SharedPtr gear_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmd_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubGearCmd_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subVelocity_;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr subPtReport_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subDesiredVelocity_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSpinMon_;
    

    rclcpp::Time vel_recv_time_;

    double desired_velocity = 0.0;
    bool auto_enabled_ = false;
    double speed_ = 0.0;
    double max_throttle_ = 0.0;
    double brake_override_ = 0.0;
    double ts_;

    // FIXME should be these first ones for running on actual car
    // int current_gear_ = 1;
    // unsigned int engine_speed_ = 0;
    // bool engine_running_ = false;
    // unsigned int shifting_counter_ = 0;
    int current_gear_ = 1;
    unsigned int engine_speed_ = 1000;
    bool engine_running_ = true;
    unsigned int shifting_counter_ = 0;
    
    double p_;
    double i_;
    double d_;
    double iMax_;
    double throttleCmdMax_;
    double throttleCmdMin_;
    double iThrottleReset_;

    double bp_;
    double bi_;
    double bd_;
    double biMax_;
    double brakeCmdMax_;
    double brakeCmdMin_;
    double iBrakeReset_;

    PID vel_pid_;
    PID brake_pid_;

}; // end of class

} // end of namespace

#endif
