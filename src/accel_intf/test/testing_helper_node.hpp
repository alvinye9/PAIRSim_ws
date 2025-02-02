/**
 * @file testing_helper_node.hpp
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

#ifndef TESTING_HELPER_NODE_HPP_
#define TESTING_HELPER_NODE_HPP_

#include <functional>
#include "acceleration_interface_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "blackandgold_msgs/msg/error_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"


typedef struct _Outputs {
    double throttle;
    rclcpp::Time last_throttle;
    double brake;
    rclcpp::Time last_brake;
    int gear;
    rclcpp::Time last_gear;
    int error;
    rclcpp::Time last_error;
    rclcpp::Time last_heartbeat;
} Outputs;


class TestAccelerationInterface : public rclcpp::Node {
public:
    TestAccelerationInterface() : Node("TestAccelerationInterface") {
        outputs.throttle = 0.0;
        outputs.brake = 0.0; 
        outputs.gear = 0; 
        outputs.error = 0;

        rclcpp::Time zero_time(0, 0, this->get_clock()->get_clock_type());
        outputs.last_throttle = zero_time;
        outputs.last_brake = zero_time;
        outputs.last_gear = zero_time;
        outputs.last_error = zero_time;

        mute = false;

        //initialize messages
        ptMsg.current_gear = 1;

        //subscribers
        subCommand = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>("/control/adrc/command", 10, std::bind(&TestAccelerationInterface::commandCallback, this, std::placeholders::_1));
        /* DEPRECATED:
        subThrottle = this->create_subscription<std_msgs::msg::Float32>("/joystick/accelerator_cmd", 10, std::bind(&TestAccelerationInterface::throttleCallback, this, std::placeholders::_1));
        subBrake = this->create_subscription<std_msgs::msg::Float32>("/joystick/brake_cmd", 10, std::bind(&TestAccelerationInterface::brakeCallback, this, std::placeholders::_1));
        subGear = this->create_subscription<std_msgs::msg::Int8>("/joystick/gear_cmd", 10, std::bind(&TestAccelerationInterface::gearCallback, this, std::placeholders::_1));
        */
        subError = this->create_subscription<blackandgold_msgs::msg::ErrorReport>("/control/errors", 10, std::bind(&TestAccelerationInterface::errorCallback, this, std::placeholders::_1));
        subHeartbeat = this->create_subscription<std_msgs::msg::Empty>("/control/accel_intf_heartbeat", 10, std::bind(&TestAccelerationInterface::heartbeatCallback, this, std::placeholders::_1));

        //publishers
        pubControlCommand = this->create_publisher<deep_orange_msgs::msg::JoystickCommand>("/control/adrc/accel_command", 10);

        /* DEPRECATED:
        pubDesiredAccel = this->create_publisher<std_msgs::msg::Float32>("/control/desired_accel", 10);
        */
        pubWheelSpeed = this->create_publisher<raptor_dbw_msgs::msg::WheelSpeedReport>("/raptor_dbw_interface/wheel_speed_report", 10);
        pubPt = this->create_publisher<deep_orange_msgs::msg::PtReport>("/raptor_dbw_interface/pt_report", 10);

        //timers
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        timer = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(10), 
                                        std::bind(&TestAccelerationInterface::sendMessages, this));

    }

    Outputs getOutputs() {
        return outputs;
    }

    void sendMessages() {
        if (mute) return;
        pubControlCommand->publish(desiredCmdMsg);
        pubWheelSpeed->publish(wheelSpeedMsg);
        pubPt->publish(ptMsg);
    }

    void setMute(bool val) {
        mute = val;
    }

    void commandCallback(deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
        outputs.throttle = msg->accelerator_cmd;
        outputs.last_throttle = this->get_clock()->now();
        outputs.brake = msg->brake_cmd;
        outputs.last_brake = this->get_clock()->now();
        outputs.gear = msg->gear_cmd;
        outputs.last_gear = this->get_clock()->now();
    }
    /* DEPRECATED:
    void throttleCallback(std_msgs::msg::Float32::SharedPtr msg) {   
        outputs.throttle = msg->data;
        outputs.last_throttle = this->get_clock()->now();
    }

    void brakeCallback(std_msgs::msg::Float32::SharedPtr msg) {
        outputs.brake = msg->data;
        outputs.last_brake = this->get_clock()->now();
    }

    void gearCallback(std_msgs::msg::Int8::SharedPtr msg) {
        outputs.gear = msg->data;
        outputs.last_gear = this->get_clock()->now();
    }*/

    void errorCallback(blackandgold_msgs::msg::ErrorReport::SharedPtr msg) {
        outputs.error = msg->severity;
        outputs.last_error = this->get_clock()->now();
    }

    void heartbeatCallback(std_msgs::msg::Empty::SharedPtr msg) {
        outputs.last_heartbeat = this->get_clock()->now();
    }

    void setDesiredAccel(float accel) {
        desiredCmdMsg.accelerator_cmd = accel;
    }

    void setEngineRpm(float rpm) {
        ptMsg.engine_rpm = rpm;
    }

    void setGear(int gear) {
        ptMsg.current_gear = gear;
    }

    void setVehicleSpeed(float speed_ms) {
        float speed_kmh = speed_ms * 3.6;
        wheelSpeedMsg.front_left = speed_kmh;
        wheelSpeedMsg.front_right = speed_kmh;
        wheelSpeedMsg.rear_left = speed_kmh;
        wheelSpeedMsg.rear_right = speed_kmh;
    }

private:
    Outputs outputs;

    bool mute;

    //messages
    deep_orange_msgs::msg::JoystickCommand desiredCmdMsg;
    raptor_dbw_msgs::msg::WheelSpeedReport wheelSpeedMsg;
    deep_orange_msgs::msg::PtReport ptMsg;
    
    //subscribers
    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr subCommand;

    /* DEPRECATED:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subThrottle;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subBrake;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subGear;
    */
    rclcpp::Subscription<blackandgold_msgs::msg::ErrorReport>::SharedPtr subError;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subHeartbeat;

    //publishers
    //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubDesiredAccel;
    rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr pubControlCommand;
    rclcpp::Publisher<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr pubWheelSpeed;
    rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr pubPt;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pubClock;

    //timers
    rclcpp::TimerBase::SharedPtr timer;
};

class ClockPub : public rclcpp::Node {
public:
    ClockPub() : Node("ClockPub") {
        pubClock = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        total_ms = 0;
    }

    void stepClock(double ms) {
        total_ms +=  ms;
        clockMsg.clock.sec = total_ms / 1000;
        clockMsg.clock.nanosec = (total_ms - clockMsg.clock.sec * 1000) * 1000000;
        pubClock->publish(clockMsg);
    }
private:

    rosgraph_msgs::msg::Clock clockMsg;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pubClock;

    double total_ms;
};

class TestExecutor : public rclcpp::executors::SingleThreadedExecutor {
public:
    TestExecutor(std::vector<rclcpp::Node::SharedPtr> nodes) : 
                        rclcpp::executors::SingleThreadedExecutor() {
        clockPub = std::make_shared<ClockPub>();

        for (auto node : nodes) {
            this->add_node(node);
        }
    }

    void runForMs(int ms) {
        double total_ms = 0.0;
        double step_size = 5.0;
        while(total_ms < ms) {
            // Not sure where this 1.11 comes from.  It is consistent, however.
            // ros2 topic hz -s reports 1.11x the expected requency with stdev ~=0 w/o.
            clockPub->stepClock(step_size/1.11); 
            total_ms += step_size;
            this->spin_some();
        }
    }
private:
    std::shared_ptr<ClockPub> clockPub;

};
void sleep_(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

#endif  // TESTING_HELPER_NODE_HPP_
