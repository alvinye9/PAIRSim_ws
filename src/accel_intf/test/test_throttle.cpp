/**
 * @file test_throttle.cpp
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

#include "gtest/gtest.h"
#include "testing_helpers.hpp"

TEST(acceleration_interface, test_throttle)
{
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    // Small positive target accel should have positive throttle and no brake
    rtObject.rtU.curr_velocity_mps = 0.0;
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 2000.0;
    rtObject.rtU.target_accel_mps2 = 2.0;
    rtObject.rtU.graceful_stop = false;
    rtObject.rtU.parking_brake = false;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS); // Some engine torque delay accounted for.  This runs the model to steady state.

    EXPECT_GT(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_LT(rtObject.rtY.throttle_pos, default_params.max_throttle_pct);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, 0.0);

    double prev_throttle_perc = rtObject.rtY.throttle_pos;

    // Increasing curr_velocity should increase throttle for given accel (to overcome drag):
    rtObject.rtU.curr_velocity_mps = 20.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_GT(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.throttle_pos, prev_throttle_perc);
    prev_throttle_perc = rtObject.rtY.throttle_pos;

    // Same target & engine RPM in higher gear should require higher throttle:
    rtObject.rtU.curr_gear = 2;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_GT(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_GT(rtObject.rtY.throttle_pos, prev_throttle_perc);

    // Large target accel with low engine RPM should have full throttle:
    rtObject.rtU.engine_rpm = 2000.0;
    rtObject.rtU.target_accel_mps2 = 10.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_EQ(rtObject.rtY.throttle_pos, default_params.max_throttle_pct);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, 0.0);

    // Target accel should be clipped based on max_accel_mps2:
    Parameters params = default_params;
    params.max_accel_mps2 = 2.0;
    setParams(rtObject, params);

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_LT(rtObject.rtY.throttle_pos, default_params.max_throttle_pct);

    // Throttle % should be clipped to max_throttle_pct:
    params = default_params;
    params.max_throttle_pct = 50.0;
    setParams(rtObject, params);

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_EQ(rtObject.rtY.throttle_pos, params.max_throttle_pct);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, 0.0);

    // Small target accel with high engine RPM should have little throttle:
    params = default_params;
    setParams(rtObject, params);

    rtObject.rtU.engine_rpm = 5000.0;
    rtObject.rtU.target_accel_mps2 = 0.5;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_GT(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_LT(rtObject.rtY.throttle_pos, params.max_throttle_pct);
    EXPECT_EQ(rtObject.rtY.p_brake_kpa, 0.0);

    // Zero target accel with high engine RPM should also have little throttle (to counteract engine-braking):
    rtObject.rtU.target_accel_mps2 = 0.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_GT(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_NEAR(rtObject.rtY.throttle_pos, 0.0, 25.0);

    // Even small negative target accel can have positive throttle:
    rtObject.rtU.target_accel_mps2 = -0.5;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_GT(rtObject.rtY.throttle_pos, 0.0);
    EXPECT_NEAR(rtObject.rtY.throttle_pos, 0.0, 25.0);

    // But, large negative target accel wont have positive throttle:
    rtObject.rtU.target_accel_mps2 = -5.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS);

    EXPECT_EQ(rtObject.rtY.throttle_pos, 0.0);

}

TEST(acceleration_interface, test_throttle_profile)
{
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    // Ramp up engine RPM from 2000 to 5000 over 2 seconds with constant target acceleration
    // Throttle should be smooth
    rtObject.rtU.curr_velocity_mps = 0.0;
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 2000.0;
    rtObject.rtU.target_accel_mps2 = 2.0;
    rtObject.rtU.graceful_stop = false;
    rtObject.rtU.parking_brake = false;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS*2); 

    int num_steps = (int) (2 / TIME_STEP_S);
    double engine_slope = (6000.0 - 2000.0) / (double) num_steps;
    double prev_throttle = rtObject.rtY.throttle_pos;
    double sum_of_squares = 0.0;
    double max_throttle_change = 0.0;
    double mean_throttle_change = 0.0;
    
    for(int i = 0; i < num_steps; i++)
    {
        rtObject.rtU.engine_rpm = 2000.0 + engine_slope * i;

        stepNTimes(rtObject, 1);

        double throttle_change = rtObject.rtY.throttle_pos - prev_throttle;
        prev_throttle = rtObject.rtY.throttle_pos;

        sum_of_squares += throttle_change * throttle_change;
        max_throttle_change = std::max(max_throttle_change, std::abs(throttle_change));
        mean_throttle_change += throttle_change;
    }

    mean_throttle_change /= num_steps;
    double stdev = std::sqrt(sum_of_squares / num_steps - mean_throttle_change * mean_throttle_change);
    
    EXPECT_NEAR(mean_throttle_change, 0.0, 0.1);
    EXPECT_LT(max_throttle_change, 0.15);
    EXPECT_NEAR(stdev, 0.0, 0.05);

    // Ramp up target acceleration from 0 to 5 over 2 seconds with constant engine RPM
    // Throttle should be strictly non-decreasing.  Smoothness constraint relaxed significantly
    // as this is not a realistic scenario (engine RPM would be increasing)
    rtObject.rtU.engine_rpm = 4000.0;
    rtObject.rtU.target_accel_mps2 = 0.0;

    stepForTime(rtObject, ENGINE_DYNAMICS_TS*2);

    num_steps = (int) (2 / TIME_STEP_S);
    double accel_slope = 5.0 / (double) num_steps;
    prev_throttle = rtObject.rtY.throttle_pos;
    sum_of_squares = 0.0;
    max_throttle_change = 0.0;
    mean_throttle_change = 0.0;

    for(int i = 0; i < num_steps; i++)
    {
        rtObject.rtU.target_accel_mps2 = accel_slope * i;
        stepNTimes(rtObject, 1);
        double throttle_change = rtObject.rtY.throttle_pos - prev_throttle;

        prev_throttle = rtObject.rtY.throttle_pos;
        EXPECT_GE(throttle_change, 0.0);

        sum_of_squares += throttle_change * throttle_change;
        max_throttle_change = std::max(max_throttle_change, std::abs(throttle_change));
        mean_throttle_change += throttle_change;
    }

    mean_throttle_change /= num_steps;
    stdev = std::sqrt(sum_of_squares / num_steps - mean_throttle_change * mean_throttle_change);

    EXPECT_NEAR(mean_throttle_change, 0.0, 0.5);
    EXPECT_LT(max_throttle_change, 1.0);
    EXPECT_NEAR(stdev, 0.0, 0.5);
}