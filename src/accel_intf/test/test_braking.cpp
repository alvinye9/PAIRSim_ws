/**
 * @file test_braking.cpp
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

TEST(acceleration_interface, test_braking) {
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    // High negative target accel should have no throttle and max brake
    rtObject.rtU.curr_velocity_mps = 0.0;
    rtObject.rtU.curr_gear = 1; 
    rtObject.rtU.engine_rpm = 0.0;
    rtObject.rtU.target_accel_mps2 = -3.0;
    rtObject.rtU.graceful_stop = false;
    rtObject.rtU.parking_brake = false;

    rtObject.step();
    
    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);
    EXPECT_LT(rtObject.rtY.p_brake_kpa, default_params.max_brake_kpa);

    double brake_kpa = rtObject.rtY.p_brake_kpa;
    double accel = calculateAccelerationFromBrake(brake_kpa, default_params);
    EXPECT_NEAR(accel, rtObject.rtU.target_accel_mps2, 0.1);

    // With engine running, should have less brake
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.curr_velocity_mps = 3.1; // Under this speed we assume no engine braking
    rtObject.rtU.engine_rpm = 2500.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS); // Some engine torque delay accounted for.  This runs the model to steady state.
    
    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);
    EXPECT_LT(rtObject.rtY.p_brake_kpa, brake_kpa);

    // With engine running, and velocity less than 3.0, engine braking should not contribute
    // This helps vehicle come to complete stop
    rtObject.rtU.curr_velocity_mps = 2.9;
    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);
    EXPECT_NEAR(rtObject.rtY.p_brake_kpa, brake_kpa, 10.0);

    // With clutch in and high velocity, should have less brake due to drag
    rtObject.rtU.engine_rpm = 0.0;
    rtObject.rtU.curr_gear = 0; // Assume clutch in to evaluate drag without engine
    rtObject.rtU.curr_velocity_mps = 20.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.p_brake_kpa, 0.0);
    EXPECT_LT(rtObject.rtY.p_brake_kpa, brake_kpa);

    double aero_drag = calculateAccelerationFromDrag(rtObject.rtU.curr_velocity_mps, default_params);
    accel = calculateAccelerationFromBrake(rtObject.rtY.p_brake_kpa, default_params);
    EXPECT_NEAR(accel+aero_drag, rtObject.rtU.target_accel_mps2, 0.1);

    // Check bounds on brake pressure
    rtObject.rtU.curr_velocity_mps = 0.0;
    Parameters params = default_params;
    params.max_decel_mps2 = -999.0;
    params.max_brake_kpa = 1000.0;
    setParams(rtObject, params);

    rtObject.rtU.target_accel_mps2 = -10.0;
    stepNTimes(rtObject, 1);

    EXPECT_EQ(rtObject.rtY.p_brake_kpa, params.max_brake_kpa);

    // Check bounds on accel
    params.max_decel_mps2 = -1.0;
    setParams(rtObject, params);
    stepNTimes(rtObject, 1);

    EXPECT_LT(rtObject.rtY.p_brake_kpa, params.max_brake_kpa);
    accel = calculateAccelerationFromBrake(rtObject.rtY.p_brake_kpa, params);
    EXPECT_NEAR(accel, params.max_decel_mps2, 0.1);
}