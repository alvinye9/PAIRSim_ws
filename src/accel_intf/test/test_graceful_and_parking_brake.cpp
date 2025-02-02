/**
 * @file test_graceful_and_parking_brake.cpp
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
#include "testing_helpers.hpp"

TEST(acceleration_interface, test_parking_brake) {
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    // Parking brake sets brake output to maximum brake pressure
    rtObject.rtU.curr_velocity_mps = 0.0;
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 0.0;
    rtObject.rtU.target_accel_mps2 = 0.0;
    rtObject.rtU.graceful_stop = false;
    rtObject.rtU.parking_brake = true;

    rtObject.step();
    
    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, default_params.max_brake_kpa);

    // Should do same with engine running
    rtObject.rtU.engine_rpm = 1000.0;

    rtObject.step();

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, default_params.max_brake_kpa);

    // And arbitrary command inputs
    rtObject.rtU.curr_velocity_mps = 10.0;
    rtObject.rtU.curr_gear = 2;
    rtObject.rtU.engine_rpm = 2000.0;
    rtObject.rtU.target_accel_mps2 = 1.0;

    rtObject.step();

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, default_params.max_brake_kpa);

    // Parking brake overrides graceful stop
    rtObject.rtU.graceful_stop = true;

    rtObject.step();

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, default_params.max_brake_kpa);
}

TEST(acceleration_interface, test_graceful_stop) {
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    // Graceful stop flag decelerates car at desired rate
    rtObject.rtU.curr_velocity_mps = 0.0;
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 0.0;
    rtObject.rtU.target_accel_mps2 = 0.0;
    rtObject.rtU.graceful_stop = true;
    rtObject.rtU.parking_brake = false;

    rtObject.step();
    
    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);

    double accel = calculateAccelerationFromBrake(rtObject.rtY.p_brake_kpa, default_params);

    EXPECT_NEAR(accel, default_params.Acc_graceful_stop_ms2, 0.01);

    // Graceful stop overrides target acceleration
    rtObject.rtU.target_accel_mps2 = 1.0;

    rtObject.step();

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);

    double accel_with_target_accel = calculateAccelerationFromBrake(rtObject.rtY.p_brake_kpa, default_params);

    EXPECT_NEAR(accel_with_target_accel, default_params.Acc_graceful_stop_ms2, 0.01);

    // Engine braking and air resistance lower required braking
    rtObject.rtU.curr_velocity_mps = 10.0;
    rtObject.rtU.engine_rpm = 3000.0;

    rtObject.step();

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);

    double accel_with_aero_and_engine = calculateAccelerationFromBrake(rtObject.rtY.p_brake_kpa, default_params);
    
    EXPECT_LT(accel_with_aero_and_engine, 0.0);
    EXPECT_LT(-accel_with_aero_and_engine, -accel);

    EXPECT_NEAR(accel_with_aero_and_engine, default_params.Acc_graceful_stop_ms2, 1.0);

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}