/**
 * @file test_acceleration_interface_node.cpp
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


#include "gtest/gtest.h"
#include "acceleration_interface_node.hpp"
#include "testing_helpers.hpp"
#include "testing_helper_node.hpp"
#include <chrono>
#include <thread>

#define NODE_INIT_TIME 100
#define NODE_RUN_TIME 50
#define NODE_RUN_TIME_S 0.05


TEST(acceleration_interface_node, test_init)
{
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("use_sim_time", true)});

    auto accel_intf_node = std::make_shared<AccelerationInterface>(node_options);
    auto test_node = std::make_shared<TestAccelerationInterface>();
    auto executor = std::make_shared<TestExecutor>(std::vector<rclcpp::Node::SharedPtr>{test_node, accel_intf_node});

    executor->runForMs(NODE_INIT_TIME);

    // auto-enable false and mute true should not send messages:
    accel_intf_node->set_parameter(rclcpp::Parameter("auto_enable", false));
    accel_intf_node->set_parameter(rclcpp::Parameter("mute", true));
    
    executor->runForMs(NODE_RUN_TIME);

    auto outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.last_error.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_gear.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_throttle.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_brake.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_heartbeat.nanoseconds(), 0);

    // auto-enable true and mute true should not send messages:
    accel_intf_node->set_parameter(rclcpp::Parameter("auto_enable", true));
    accel_intf_node->set_parameter(rclcpp::Parameter("mute", true));

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.last_error.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_gear.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_throttle.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_brake.nanoseconds(), 0);
    EXPECT_EQ(outputs.last_heartbeat.nanoseconds(), 0);

    // auto-enable true and mute false should publish messages:
    accel_intf_node->set_parameter(rclcpp::Parameter("auto_enable", true));
    accel_intf_node->set_parameter(rclcpp::Parameter("mute", false));

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.last_error.nanoseconds(), 0); // should be no errors

    double now = accel_intf_node->get_clock()->now().seconds();
    EXPECT_NEAR(outputs.last_gear.seconds(), now, 0.01);
    EXPECT_NEAR(outputs.last_throttle.seconds(), now, 0.01);
    EXPECT_NEAR(outputs.last_brake.seconds(), now, 0.01);
    EXPECT_NEAR(outputs.last_heartbeat.seconds(), now, 0.01);


}

TEST(acceleration_interface_node, test_general_function)
{    
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("use_sim_time", true),
            rclcpp::Parameter("mute", false)});

    auto accel_intf_node = std::make_shared<AccelerationInterface>(node_options);
    auto test_node = std::make_shared<TestAccelerationInterface>();
    auto executor = std::make_shared<TestExecutor>(std::vector<rclcpp::Node::SharedPtr>{test_node, accel_intf_node});

    executor->runForMs(NODE_INIT_TIME);
    accel_intf_node->set_parameter(rclcpp::Parameter("auto_enable", true));

    // Negative accel should brake:
    test_node->setDesiredAccel(-5.0);
    test_node->setGear(1);
    test_node->setEngineRpm(2000.0);
    test_node->setVehicleSpeed(5.0);

    executor->runForMs(NODE_RUN_TIME);

    Outputs outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.last_error.nanoseconds(), 0);
    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_GT(outputs.brake, 0.0);
    EXPECT_EQ(outputs.gear, 1);

    // Positive accel should cause throttle:
    test_node->setDesiredAccel(5.0);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.last_error.nanoseconds(), 0);
    EXPECT_GT(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, 0.0);
    EXPECT_EQ(outputs.gear, 1);

    // 0 accel does not necessarily mean 0 throttle due to aero/engine braking:
    test_node->setDesiredAccel(0.0);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_GT(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, 0.0);
    EXPECT_EQ(outputs.gear, 1);

    // Gear should change if RPM goes above upper bound:
    double gear_shift_delay = accel_intf_node->get_parameter("Min_shift_delay_s").as_double() * 1000;
    executor->runForMs(gear_shift_delay); // Must wait for shift delay after initialization of node

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.gear, 1);

    test_node->setEngineRpm(accel_intf_node->get_parameter("gearset.gear1_ub_rpm").as_double() + 100.0);
    while(outputs.gear == 1) {
        executor->runForMs(5);
        outputs = test_node->getOutputs();
    }

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.gear, 2);

    // Gear should downshift after correct amount of time has passed
    test_node->setEngineRpm(accel_intf_node->get_parameter("gearset.gear2_lb_rpm").as_double() - 100.0);
    test_node->setGear(2);

    int ms_to_shift = 0;
    while(outputs.gear == 2) {
        executor->runForMs(10);
        outputs = test_node->getOutputs();
        ms_to_shift += 10;
    }

    EXPECT_NEAR(ms_to_shift, gear_shift_delay, NODE_RUN_TIME); 
    EXPECT_EQ(outputs.gear, 1);
}

TEST(acceleration_interface_node, test_parameters)
{    
    double test_gear1_ub_rpm = 1000.0;
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("use_sim_time", true),
            rclcpp::Parameter("mute", false)});

    auto accel_intf_node = std::make_shared<AccelerationInterface>(node_options);
    auto test_node = std::make_shared<TestAccelerationInterface>();
    auto executor = std::make_shared<TestExecutor>(std::vector<rclcpp::Node::SharedPtr>{test_node, accel_intf_node});

    executor->runForMs(NODE_INIT_TIME);
    accel_intf_node->set_parameter(rclcpp::Parameter("auto_enable", true));

    // Max throttle percentage:
    double max_accel_mps2 = accel_intf_node->get_parameter("max_accel_mps2").as_double();
    test_node->setDesiredAccel(max_accel_mps2);
    test_node->setGear(1);
    test_node->setEngineRpm(2000.0);
    test_node->setVehicleSpeed(2.0);

    executor->runForMs(NODE_RUN_TIME);

    Outputs outputs = test_node->getOutputs();
    double max_throttle_pct = accel_intf_node->get_parameter("max_throttle_pct").as_double();
    EXPECT_EQ(outputs.throttle, max_throttle_pct);

    accel_intf_node->set_parameter(rclcpp::Parameter("max_throttle_pct", max_throttle_pct / 2));

    int node_param_time = accel_intf_node->get_parameter("ts_param").as_double() * 1000 + 100;
    executor->runForMs(node_param_time);

    outputs = test_node->getOutputs();
    EXPECT_EQ(accel_intf_node->get_parameter("max_throttle_pct").as_double(), max_throttle_pct / 2);
    EXPECT_EQ(outputs.throttle, accel_intf_node->get_parameter("max_throttle_pct").as_double());

    // Max accel:
    accel_intf_node->set_parameter(rclcpp::Parameter("max_accel_mps2", max_accel_mps2 / 4));

    executor->runForMs(node_param_time);

    outputs = test_node->getOutputs();
    EXPECT_EQ(accel_intf_node->get_parameter("max_accel_mps2").as_double(), max_accel_mps2 / 4);
    EXPECT_LT(outputs.throttle, accel_intf_node->get_parameter("max_throttle_pct").as_double());

    // Max brake pressure:
    double max_decel_mps2 = accel_intf_node->get_parameter("max_decel_mps2").as_double();
    test_node->setDesiredAccel(max_decel_mps2);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    double test_max_brake_val = outputs.brake / 2;

    accel_intf_node->set_parameter(rclcpp::Parameter("max_brake_kpa", test_max_brake_val));

    executor->runForMs(node_param_time);

    outputs = test_node->getOutputs();
    EXPECT_EQ(accel_intf_node->get_parameter("max_brake_kpa").as_double(), test_max_brake_val);
    EXPECT_EQ(outputs.brake, accel_intf_node->get_parameter("max_brake_kpa").as_double());

    // Max decel:
    accel_intf_node->set_parameter(rclcpp::Parameter("max_decel_mps2", max_decel_mps2 / 4));

    executor->runForMs(node_param_time);

    outputs = test_node->getOutputs();
    EXPECT_EQ(accel_intf_node->get_parameter("max_decel_mps2").as_double(), max_decel_mps2 / 4);
    EXPECT_LT(outputs.brake, accel_intf_node->get_parameter("max_brake_kpa").as_double());


}

TEST(acceleration_interface_node, test_graceful_stop)
{    
    double test_gear1_ub_rpm = 1000.0;
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(std::vector<rclcpp::Parameter>{
            rclcpp::Parameter("use_sim_time", true),
            rclcpp::Parameter("mute", false)});

    auto accel_intf_node = std::make_shared<AccelerationInterface>(node_options);
    auto test_node = std::make_shared<TestAccelerationInterface>();
    auto executor = std::make_shared<TestExecutor>(std::vector<rclcpp::Node::SharedPtr>{test_node, accel_intf_node});

    executor->runForMs(NODE_INIT_TIME);
    accel_intf_node->set_parameter(rclcpp::Parameter("auto_enable", true));

    // Parking brake should be enabled at car startup:
    test_node->setDesiredAccel(0.0);
    test_node->setGear(1);
    test_node->setEngineRpm(2000.0);
    test_node->setVehicleSpeed(0.0);

    executor->runForMs(NODE_RUN_TIME);

    Outputs outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, accel_intf_node->get_parameter("max_brake_kpa").as_double());

    // should get errors if we stop publishing after max_msg_timeout_s:
    test_node->setDesiredAccel(5.0);
    test_node->setVehicleSpeed(15.0);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.last_error.nanoseconds(), 0);
    EXPECT_GT(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, 0.0);

    test_node->setMute(true);
    double max_msg_timeout_ms = accel_intf_node->get_parameter("max_msg_timeout_s").as_double()*1000;

    int ms_to_timeout = 0;
    while(outputs.last_error.nanoseconds() == 0) {
        executor->runForMs(10);
        outputs = test_node->getOutputs();
        ms_to_timeout += 10;
    }

    EXPECT_NEAR(ms_to_timeout, max_msg_timeout_ms, 2*NODE_RUN_TIME);
    double now = accel_intf_node->get_clock()->now().seconds();
    EXPECT_NEAR(outputs.last_error.seconds(), now, NODE_RUN_TIME_S);
    EXPECT_EQ(outputs.error, 5);

    // Graceful stop should be triggered if we are getting errors:

    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_GT(outputs.brake, 0.0);

    // Graceful stop should not unlatch unless speed goes below 1.0:
    test_node->setMute(false);

    executor->runForMs(2*NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    now = accel_intf_node->get_clock()->now().seconds();
    EXPECT_LT(outputs.last_error.seconds(), now - NODE_RUN_TIME_S);
    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_GT(outputs.brake, 0.0);

    test_node->setVehicleSpeed(0.5);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    now = accel_intf_node->get_clock()->now().seconds();
    EXPECT_LT(outputs.last_error.seconds(), now - NODE_RUN_TIME_S);
    EXPECT_GT(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, 0.0);

    // Parking brake should be enabled if speed goes below 1.0 and we are trying to decelerate
    test_node->setDesiredAccel(-1.0);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, accel_intf_node->get_parameter("max_brake_kpa").as_double());

    // Parking brake should be enabled if speed is below 1.0 and desired acceleration is small and positive:
    test_node->setDesiredAccel(0.05);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, accel_intf_node->get_parameter("max_brake_kpa").as_double());

    // Parking brake should be enabled if speed goes below 1.0 and 
    // graceful stop is triggered despite a high acceleration:
    test_node->setDesiredAccel(5.0);

    executor->runForMs(NODE_RUN_TIME);

    outputs = test_node->getOutputs();
    EXPECT_GT(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, 0.0);

    test_node->setMute(true);

    now = accel_intf_node->get_clock()->now().seconds();
    while(outputs.last_error.seconds() < now - NODE_RUN_TIME_S) {
        executor->runForMs(10);
        now = accel_intf_node->get_clock()->now().seconds();
        outputs = test_node->getOutputs();
    }

    outputs = test_node->getOutputs();
    now = accel_intf_node->get_clock()->now().seconds();
    EXPECT_NEAR(outputs.last_error.seconds(), now, NODE_RUN_TIME_S);
    EXPECT_EQ(outputs.throttle, 0.0);
    EXPECT_EQ(outputs.brake, accel_intf_node->get_parameter("max_brake_kpa").as_double());


}


int main(int argc, char **argv) {
    rclcpp::init(0, nullptr);
    testing::InitGoogleTest(&argc, argv);
    bool ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}