/**
 * @file acceleration_interface_node.hpp
 * @author: Ethan Brown, Yuchen Song
 * @date: 12-26-2023
 *
 * Copyright (c) 2023-2024, Ethan Brown, Yuchen Song
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ACCELERATION_INTERFACE_NODE_HPP_
#define ACCELERATION_INTERFACE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "acceleration_interface.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/empty.hpp"

#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/joystick_command.hpp"
#include "blackandgold_msgs/msg/error_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"

class AccelerationInterface : public rclcpp::Node
{
public:
    AccelerationInterface(rclcpp::NodeOptions node_options);

private:
    void controlCallback();
    void receiveWheelSpeedCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
    void receivePTCallback(const deep_orange_msgs::msg::PtReport::SharedPtr msg);
    void receiveDesAccelCallback(const std_msgs::msg::Float32::SharedPtr msg); // for receiving as float
    void receiveDesAccelCallback_passthrough(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg); // for passthrough mode receiving as JoystickCommand
    void updateParams();
    void declareParams();

    void publishThrottleBrakeGear();
    void checkMsgTimeouts();
    void checkParkingBrake();


    // Clock and Timers
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr param_timer_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubHeartbeat_;
    rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr pubCommand_;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubSteeringCmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmd_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubGearCmd_;
    rclcpp::Publisher<blackandgold_msgs::msg::ErrorReport>::SharedPtr pubError_;

    // Subscribers
    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr subVelocity_;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr subPT_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subDesAccel_;
    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr subControlCmd_;

    // timesteps
    float ts_control_;
    float ts_param_;

    // Timeouts
    rclcpp::Time last_velocity_time_;
    rclcpp::Time last_pt_time_;
    rclcpp::Time last_des_accel_time_;
    bool message_timeout_;

    // Inputs
    float curr_velocity_;
    int curr_gear_;
    float engine_rpm_;
    float target_accel_;
    bool graceful_stop_;
    bool parking_brake_;

    // Outputs
    float throttle_cmd_;
    float brake_cmd_;
    int gear_cmd_;

    // heartbeat message
    std_msgs::msg::Empty heartbeat_msg_;

    // command messages
    deep_orange_msgs::msg::JoystickCommand command_msg_;
std_msgs::msg::Float32 steering_cmd_msg_;
    std_msgs::msg::Float32 throttle_cmd_msg_;
    std_msgs::msg::Float32 brake_cmd_msg_;
    std_msgs::msg::Int8 gear_cmd_msg_;

    // simulink model
    acceleration_interface rtObject_;
};

#endif  // ACCELERATION_INTERFACE_NODE_HPP_