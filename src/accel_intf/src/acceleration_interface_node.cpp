/**
 * @file acceleration_interface_node.cpp
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

#include <cmath>
#include "acceleration_interface_node.hpp"
#include "acceleration_interface.h"

AccelerationInterface::AccelerationInterface(rclcpp::NodeOptions node_options) : Node("AccelerationInterface", node_options) {
    //publishers
    pubHeartbeat_ = this->create_publisher<std_msgs::msg::Empty>("/control/accel_intf_heartbeat", 1);

    // replaced by above publisher (but kept for debug + future possible uses)
    pubCommand_ = this->create_publisher<deep_orange_msgs::msg::JoystickCommand>("/control/controller/command",1);
pubSteeringCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd",1);
    pubThrottleCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/throttle_cmd",1);
    pubBrakeCmd_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 1);
    pubGearCmd_ = this->create_publisher<std_msgs::msg::Int8>("/joystick/gear_cmd", 1);
    pubError_ = this->create_publisher<blackandgold_msgs::msg::ErrorReport>("/control/errors", 1);

    //subscribers
    //TODO: Potentially use velocity from the localization interface instead of the wheel speed report
    //TODO: qos?
    subVelocity_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>("/raptor_dbw_interface/wheel_speed_report", 1, 
                    std::bind(&AccelerationInterface::receiveWheelSpeedCallback, this, std::placeholders::_1));

    subPT_ = this->create_subscription<deep_orange_msgs::msg::PtReport>("/raptor_dbw_interface/pt_report", 1,
                    std::bind(&AccelerationInterface::receivePTCallback, this, std::placeholders::_1));


    subControlCmd_ = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>("/control/controller/accel_command", rclcpp::SensorDataQoS(),
                    std::bind(&AccelerationInterface::receiveDesAccelCallback_passthrough, this, std::placeholders::_1));

    // replaced by above subscriber (but kept for debug + future possible uses)
    // subDesAccel_ = this->create_subscription<std_msgs::msg::Float32>("/control/desired_acceleration", 1,
                    // std::bind(&AccelerationInterface::receiveDesAccelCallback, this, std::placeholders::_1));
    


    //initialize simulink model
    rtObject_.initialize();

    declareParams();
    updateParams();

    //initialize clock
    if (this->get_parameter("use_sim_time").as_bool()) {
        clock_ = this->get_clock();
    } else {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    }

    //timers
    ts_control_ = 0.01;
    int ts_control_ms = static_cast<int>(ts_control_ * 1000.0);
    control_timer_ = rclcpp::create_timer(this, clock_, std::chrono::milliseconds(ts_control_ms),
                                            std::bind(&AccelerationInterface::controlCallback, this));

    ts_param_ = this->get_parameter("ts_param").as_double();
    int ts_param_ms = static_cast<int>(ts_param_ * 1000.0);
    param_timer_ = rclcpp::create_timer(this, clock_, std::chrono::milliseconds(ts_param_ms),
                                            std::bind(&AccelerationInterface::updateParams, this));

    //initialize timeouts
    last_velocity_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    last_pt_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    last_des_accel_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

    //initialize output messages
    throttle_cmd_msg_.data = 0.0;
    brake_cmd_msg_.data = 0.0;
    gear_cmd_msg_.data = 1;

    //initialize flags
    graceful_stop_ = false;
    parking_brake_ = true;
    message_timeout_ = false;

}

void AccelerationInterface::declareParams()
{
    //config modifiable parameters
    this->declare_parameter<float>("ts_param", 1.0);
    this->declare_parameter<float>("A_vehicle_m2", 1.0);
    this->declare_parameter<float>("A_caliper_mm2", 4448.0);
    this->declare_parameter<float>("Coef_drag", 0.3);
    this->declare_parameter<float>("M_vehicle_kg", 787.0);
    this->declare_parameter<float>("Mue_k_brake", 0.45);
    this->declare_parameter<float>("R_brake_lever_m", 0.134);
    this->declare_parameter<float>("R_tire_fr_m", 0.3);
    this->declare_parameter<float>("R_tire_re_m", 0.31);
    this->declare_parameter<float>("Ratio_diff_re", 3.0);
    this->declare_parameter<float>("Roh_air", 1.22);
    this->declare_parameter<float>("accel_graceful_stop_mps2", -5.0);
    this->declare_parameter<float>("max_msg_timeout_s", 0.5);
    this->declare_parameter<float>("Min_shift_delay_s", 2.0);
    this->declare_parameter<float>("Shift_timeout_s", 0.7);
    
    //gearset parameters
    this->declare_parameter<float>("gearset.gear1_lb_rpm", 0.0);
    this->declare_parameter<float>("gearset.gear1_ub_rpm", 3900.0);
    this->declare_parameter<float>("gearset.gear2_lb_rpm", 2000.0);
    this->declare_parameter<float>("gearset.gear2_ub_rpm", 4500.0);
    this->declare_parameter<float>("gearset.gear3_lb_rpm", 2815.0);
    this->declare_parameter<float>("gearset.gear3_ub_rpm", 4800.0);
    this->declare_parameter<float>("gearset.gear4_lb_rpm", 3375.0);
    this->declare_parameter<float>("gearset.gear4_ub_rpm", 5000.0);
    this->declare_parameter<float>("gearset.gear5_lb_rpm", 3785.0);
    this->declare_parameter<float>("gearset.gear5_ub_rpm", 4800.0);
    this->declare_parameter<float>("gearset.gear6_lb_rpm", 3980.0);
    this->declare_parameter<float>("gearset.gear6_ub_rpm", 600.0);

    //runtime modifiable parameters
    this->declare_parameter<float>("max_throttle_pct", 50.0);
    this->declare_parameter<float>("max_brake_kpa", 1400.0);
    this->declare_parameter<float>("max_accel_mps2", 3.0);
    this->declare_parameter<float>("max_decel_mps2", -6.0);
    this->declare_parameter<bool>("mute", true);
    this->declare_parameter<bool>("auto_enable", false);
}

void AccelerationInterface::updateParams() 
{
    // Don't modify these parameters at runtime
    static bool first_call = true;
    if (first_call) {
        rtObject_.rtP.A_Vehicle_m2 = this->get_parameter("A_vehicle_m2").as_double();
        rtObject_.rtP.A_caliper_mm2 = this->get_parameter("A_caliper_mm2").as_double();
        rtObject_.rtP.Coef_drag = this->get_parameter("Coef_drag").as_double();
        rtObject_.rtP.M_vehicle_kg = this->get_parameter("M_vehicle_kg").as_double();
        rtObject_.rtP.Mue_k_brake = this->get_parameter("Mue_k_brake").as_double();
        rtObject_.rtP.R_brake_lever_m = this->get_parameter("R_brake_lever_m").as_double();
        rtObject_.rtP.R_tire_fr = this->get_parameter("R_tire_fr_m").as_double();
        rtObject_.rtP.R_tire_re = this->get_parameter("R_tire_re_m").as_double();
        rtObject_.rtP.Ratio_diff_re = this->get_parameter("Ratio_diff_re").as_double();
        rtObject_.rtP.roh_air = this->get_parameter("Roh_air").as_double();
        rtObject_.rtP.Acc_graceful_stop_ms2 = this->get_parameter("accel_graceful_stop_mps2").as_double();
        rtObject_.rtP.Min_shift_delay_s = this->get_parameter("Min_shift_delay_s").as_double();
        rtObject_.rtP.Shift_timeout_s = this->get_parameter("Shift_timeout_s").as_double();
        
        rtObject_.rtP.Gearset.n_lb_rpm_1 = this->get_parameter("gearset.gear1_lb_rpm").as_double();
        rtObject_.rtP.Gearset.n_ub_rpm_1 = this->get_parameter("gearset.gear1_ub_rpm").as_double();
        rtObject_.rtP.Gearset.n_lb_rpm_2 = this->get_parameter("gearset.gear2_lb_rpm").as_double();
        rtObject_.rtP.Gearset.n_ub_rpm_2 = this->get_parameter("gearset.gear2_ub_rpm").as_double();
        rtObject_.rtP.Gearset.n_lb_rpm_3 = this->get_parameter("gearset.gear3_lb_rpm").as_double();
        rtObject_.rtP.Gearset.n_ub_rpm_3 = this->get_parameter("gearset.gear3_ub_rpm").as_double();
        rtObject_.rtP.Gearset.n_lb_rpm_4 = this->get_parameter("gearset.gear4_lb_rpm").as_double();
        rtObject_.rtP.Gearset.n_ub_rpm_4 = this->get_parameter("gearset.gear4_ub_rpm").as_double();
        rtObject_.rtP.Gearset.n_lb_rpm_5 = this->get_parameter("gearset.gear5_lb_rpm").as_double();
        rtObject_.rtP.Gearset.n_ub_rpm_5 = this->get_parameter("gearset.gear5_ub_rpm").as_double();
        rtObject_.rtP.Gearset.n_lb_rpm_6 = this->get_parameter("gearset.gear6_lb_rpm").as_double();
        rtObject_.rtP.Gearset.n_ub_rpm_6 = this->get_parameter("gearset.gear6_ub_rpm").as_double();

        first_call = false;
    }

    rtObject_.rtP.Throttle_max_p = this->get_parameter("max_throttle_pct").as_double();
    rtObject_.rtP.Brake_max_kpa = this->get_parameter("max_brake_kpa").as_double();
    rtObject_.rtP.Acc_max_ms2 = this->get_parameter("max_accel_mps2").as_double();
    rtObject_.rtP.Acc_min_ms2 = this->get_parameter("max_decel_mps2").as_double();
}


void AccelerationInterface::receiveWheelSpeedCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
    const double kphToMps = 1.0 / 3.6;
    double rear_left = msg->rear_left;
    double rear_right = msg->rear_right;

    curr_velocity_ = (rear_left + rear_right) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
    last_velocity_time_ = this->clock_->now();
}

void AccelerationInterface::receivePTCallback(const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
    engine_rpm_ = msg->engine_rpm;
    curr_gear_ = msg->current_gear;
    last_pt_time_ = this->clock_->now();
}

// accel callback receive for passthrough mode
void AccelerationInterface::receiveDesAccelCallback_passthrough(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {

    last_des_accel_time_ = this->clock_->now();

    // peel off accel command
    target_accel_ = msg->accelerator_cmd;
    
    // passthrough the rest of the fields (with the exception of gear + brake since we set those)
    command_msg_.stamp = msg->stamp;
    command_msg_.counter = msg->counter;
    command_msg_.emergency_stop = msg->emergency_stop;
    command_msg_.joy_enable = msg->joy_enable;

    // steering passthrough
    command_msg_.steering_cmd = msg->steering_cmd;
steering_cmd_msg_.data = msg->steering_cmd;
    

}

// accel callback receive for float32 mode
void AccelerationInterface::receiveDesAccelCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    target_accel_ = msg->data;
    last_des_accel_time_ = this->clock_->now();
}

void AccelerationInterface::checkParkingBrake() {
    // Parking brake if we are less than 1m/s and 1+ of the following:
    // 1. We are in graceful stop mode (graceful stop resets <1m/s if fault is cleared)
    // 2. We are trying to slow down
    // 3. The engine is off (RPM is lower than idle)
    // TODO: Should we replace target_accel_ with desired_velocity?
    if ((curr_velocity_ < 1.0) && 
        (graceful_stop_ || target_accel_ < 0.1 || engine_rpm_ < 500.0)){
        parking_brake_ = true;
    } else {
        parking_brake_ = false;
    }
}

void AccelerationInterface::controlCallback() {
    if(!this->get_parameter("auto_enable").as_bool()) return;

    checkMsgTimeouts();
    checkParkingBrake();

    //set model inputs
    rtObject_.rtU.curr_velocity_mps = curr_velocity_;
    rtObject_.rtU.curr_gear = curr_gear_;
    rtObject_.rtU.engine_rpm = engine_rpm_;
    rtObject_.rtU.target_accel_mps2 = target_accel_;
    rtObject_.rtU.graceful_stop = graceful_stop_;
    rtObject_.rtU.parking_brake = parking_brake_;

    //step the model
    rtObject_.step();

    //get model outputs
    throttle_cmd_ = rtObject_.rtY.throttle_pos;
    brake_cmd_ = rtObject_.rtY.p_brake_kpa;
    gear_cmd_ = rtObject_.rtY.gear_out;

    //check for NaNs
    //if we have NaNs, set throttle to 0, brake to max, and gear to 1
    if(std::isnan(throttle_cmd_) || std::isnan(brake_cmd_) || std::isnan(gear_cmd_)) {
        throttle_cmd_ = 0;
        brake_cmd_ = this->get_parameter("max_brake_kpa").as_double();
        gear_cmd_ = 1;
    }

    // throttle_cmd_ = 0;
    // brake_cmd_ = 10;
    // gear_cmd_ = 1;
    //publish outputs

    command_msg_.accelerator_cmd = throttle_cmd_;
    command_msg_.brake_cmd = brake_cmd_;
    command_msg_.gear_cmd = gear_cmd_;


    throttle_cmd_msg_.data = throttle_cmd_;
    brake_cmd_msg_.data = brake_cmd_;
    gear_cmd_msg_.data = gear_cmd_;

    publishThrottleBrakeGear();
}

void AccelerationInterface::publishThrottleBrakeGear() {
    if(this->get_parameter("mute").as_bool()) return;


    pubCommand_->publish(command_msg_);
    pubHeartbeat_->publish(heartbeat_msg_);

    pubSteeringCmd_->publish(steering_cmd_msg_);
    pubThrottleCmd_->publish(throttle_cmd_msg_);
    pubBrakeCmd_->publish(brake_cmd_msg_);
    pubGearCmd_->publish(gear_cmd_msg_);
    
}

void AccelerationInterface::checkMsgTimeouts() {
    // TODO: Need to audit parking/graceful stop logic.
    message_timeout_ = false;

    blackandgold_msgs::msg::ErrorReport error_msg;
    error_msg.header.stamp = this->clock_->now();
    error_msg.origin = "acceleration_interface";
    error_msg.module = "control";
    error_msg.lifetime = 1.0; // TODO: Should this be 1/100? Should it latch? If this latches, remove the latching in the graceful stop.

    double max_msg_timeout_s = this->get_parameter("max_msg_timeout_s").as_double();

    if ((this->clock_->now() - last_velocity_time_).seconds() > max_msg_timeout_s) {
        message_timeout_ = true;
        error_msg.description = "Timeout on wheel speeds message! Latching graceful stop!";
        error_msg.severity = blackandgold_msgs::msg::ErrorReport::ERROR_FATAL;
    }
    if ((this->clock_->now() - last_pt_time_).seconds() > max_msg_timeout_s) {
        message_timeout_ = true;
        error_msg.description = "Timeout on PT report message! Latching graceful stop!";
        error_msg.severity = blackandgold_msgs::msg::ErrorReport::ERROR_FATAL;
    }
    if ((this->clock_->now() - last_des_accel_time_).seconds() > max_msg_timeout_s) {
        message_timeout_ = true;
        error_msg.description = "Timeout on desired acceleration message! Latching graceful stop!";
        error_msg.severity = blackandgold_msgs::msg::ErrorReport::ERROR_FATAL;
    }

    // Unlatch graceful stop if we've slowed down
    if(curr_velocity_ < 1.0) {
        graceful_stop_ = false;
    }

    // Latch graceful stop if we've timed out
    if(message_timeout_) {
        graceful_stop_ = true;
        pubError_->publish(error_msg);
    }
}
