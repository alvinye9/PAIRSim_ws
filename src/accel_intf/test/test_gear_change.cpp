/**
 * @file test_gear_change.cpp
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

TEST(acceleration_interface, test_default_params)
{
    // Test that post-shift RPM is higher than that gear's lower bound RPM
    // This can only test data dictionary parameters, not those set in ROS2
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    double prev_gear_UB;
    double prev_gear_ratio;
    double post_shift_rpm;
    double curr_gear_ratio;
    double curr_gear_lb;
    
    for (int i = 1; i < 6; i++)
    {
        switch(i) {
            case 1:
                prev_gear_UB = rtObject.rtP.Gearset.n_ub_rpm_1;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_1;
                curr_gear_lb = rtObject.rtP.Gearset.n_lb_rpm_2;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_2;
                break;
            case 2:
                prev_gear_UB = rtObject.rtP.Gearset.n_ub_rpm_2;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_2;
                curr_gear_lb = rtObject.rtP.Gearset.n_lb_rpm_3;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_3;
                break;
            case 3:
                prev_gear_UB = rtObject.rtP.Gearset.n_ub_rpm_3;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_3;
                curr_gear_lb = rtObject.rtP.Gearset.n_lb_rpm_4;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_4;
                break;
            case 4:
                prev_gear_UB = rtObject.rtP.Gearset.n_ub_rpm_4;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_4;
                curr_gear_lb = rtObject.rtP.Gearset.n_lb_rpm_5;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_5;
                break;
            case 5:
                prev_gear_UB = rtObject.rtP.Gearset.n_ub_rpm_5;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_5;
                curr_gear_lb = rtObject.rtP.Gearset.n_lb_rpm_6;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_6;
                break;
        }
        post_shift_rpm = prev_gear_UB * curr_gear_ratio / prev_gear_ratio;
        EXPECT_GT(post_shift_rpm, curr_gear_lb);
    }
}

TEST(acceleration_interface, test_gear_change)
{
    acceleration_interface rtObject;
    initializeAccelerationInterface(rtObject, default_params);

    rtObject.rtU.curr_velocity_mps = 0.0;
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.target_accel_mps2 = 0.0;
    rtObject.rtU.graceful_stop = false;
    rtObject.rtU.parking_brake = false;
    stepForTime(rtObject, 2.0); // Gear shift not possible until after Min_shift_delay_s

    // If engine RPM goes above upper bound of current gear, should shift up

    rtObject.rtU.engine_rpm = default_params.Gearset_n_ub_rpm_1 + 100.0;
    stepNTimes(rtObject, 1);
    EXPECT_EQ(rtObject.rtY.gear_out, 2);
    stepForTime(rtObject, 0.5); 
    EXPECT_EQ(rtObject.rtY.gear_out, 2);
    rtObject.rtU.curr_gear = rtObject.rtY.gear_out; // Gear shift has happened
    stepForTime(rtObject, 1.5);
    EXPECT_EQ(rtObject.rtY.gear_out, 2);

    // If engine RPM goes below lower bound of current gear, should shift down
    rtObject.rtU.engine_rpm = default_params.Gearset_n_lb_rpm_2 - 100.0;
    stepNTimes(rtObject, 1);
    EXPECT_EQ(rtObject.rtY.gear_out, 1);
    stepForTime(rtObject, 0.5);
    EXPECT_EQ(rtObject.rtY.gear_out, 1);
    rtObject.rtU.curr_gear = rtObject.rtY.gear_out;
    stepForTime(rtObject, 1.5);
    EXPECT_EQ(rtObject.rtY.gear_out, 1);

    // Gear shifts should not happen sooner than 1.2s apart
    // Testing with unrealistic case of very high RPM
    double shift_delay = default_params.Min_shift_delay_s;
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 2000;
    stepForTime(rtObject, 2.0); // reset
    EXPECT_EQ(rtObject.rtU.curr_gear, 1);

    rtObject.rtU.engine_rpm = 10000;
    while(rtObject.rtY.gear_out == 1) {
        stepNTimes(rtObject, 1);
    }

    EXPECT_EQ(rtObject.rtY.gear_out, 2);
    rtObject.rtU.curr_gear = rtObject.rtY.gear_out;

    double s_to_shift = 0;

    while(rtObject.rtY.gear_out == 2) {
        stepNTimes(rtObject, 1);
        s_to_shift += TIME_STEP_S;
    }

    EXPECT_EQ(rtObject.rtY.gear_out, 3);
    EXPECT_NEAR(s_to_shift, shift_delay, 2*TIME_STEP_S);

    // run through all upshifts
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 2000;
    stepForTime(rtObject, 2.0); // reset

    for(int i = 1; i < 6; i++)
    {
        double curr_gear_ub;
        double curr_gear_ratio;
        double prev_gear_ratio;
        double prev_gear_ub;
        switch(i) {
            case 1:
                curr_gear_ub = default_params.Gearset_n_ub_rpm_1;
                curr_gear_ratio = 1.0;
                prev_gear_ratio = 1.0;
                prev_gear_ub = default_params.Gearset_n_lb_rpm_1;
                break;
            case 2:
                curr_gear_ub = default_params.Gearset_n_ub_rpm_2;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_2;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_1;
                prev_gear_ub = default_params.Gearset_n_ub_rpm_1;
                break;
            case 3:
                curr_gear_ub = default_params.Gearset_n_ub_rpm_3;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_3;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_2;
                prev_gear_ub = default_params.Gearset_n_ub_rpm_2;
                break;
            case 4:
                curr_gear_ub = default_params.Gearset_n_ub_rpm_4;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_4;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_3;
                prev_gear_ub = default_params.Gearset_n_ub_rpm_3;
                break;
            case 5:
                curr_gear_ub = default_params.Gearset_n_ub_rpm_5;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_5;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_4;
                prev_gear_ub = default_params.Gearset_n_ub_rpm_4;
                break;
        }
        double new_engine_rpm = prev_gear_ub * curr_gear_ratio / prev_gear_ratio;
        double engine_rpm_slope = (curr_gear_ub - new_engine_rpm) / (2.0 / TIME_STEP_S); // Aim to shift in 2 seconds
        rtObject.rtU.engine_rpm = new_engine_rpm;
        rtObject.rtU.curr_gear = i;
    
        for(int j = 0; j < 3/TIME_STEP_S; j++)
        {
            rtObject.rtU.engine_rpm += engine_rpm_slope;
            stepNTimes(rtObject, 1);

            if(rtObject.rtY.gear_out != i)
            {
                break;
            }
        }
        EXPECT_EQ(rtObject.rtY.gear_out, i+1);
        EXPECT_NEAR(rtObject.rtU.engine_rpm, curr_gear_ub, 25);
        stepForTime(rtObject, default_params.Shift_timeout_s / 2.0); // Wait for gear shift to complete
        
    }

    // run through all downshifts
    rtObject.rtU.curr_gear = 6;
    rtObject.rtU.engine_rpm = rtObject.rtP.Gearset.n_ub_rpm_6;
    stepForTime(rtObject, 2.0);

    for(int i = 6; i > 0; i--)
    {
        double curr_gear_lb;
        double curr_gear_ratio;
        double prev_gear_ratio;
        double prev_gear_lb;
        switch(i) {
            case 1:
                curr_gear_lb = default_params.Gearset_n_lb_rpm_1;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_1;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_2;
                prev_gear_lb = default_params.Gearset_n_lb_rpm_2;
                break;
            case 2:
                curr_gear_lb = default_params.Gearset_n_lb_rpm_2;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_2;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_3;
                prev_gear_lb = default_params.Gearset_n_lb_rpm_3;
                break;
            case 3:
                curr_gear_lb = default_params.Gearset_n_lb_rpm_3;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_3;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_4;
                prev_gear_lb = default_params.Gearset_n_lb_rpm_4;
                break;
            case 4:
                curr_gear_lb = default_params.Gearset_n_lb_rpm_4;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_4;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_5;
                prev_gear_lb = default_params.Gearset_n_lb_rpm_5;
                break;
            case 5:
                curr_gear_lb = default_params.Gearset_n_lb_rpm_5;
                curr_gear_ratio = rtObject.rtP.Gearset.i_gearRat_5;
                prev_gear_ratio = rtObject.rtP.Gearset.i_gearRat_6;
                prev_gear_lb = default_params.Gearset_n_lb_rpm_6;
                break;
            case 6:
                curr_gear_lb = default_params.Gearset_n_lb_rpm_6;
                curr_gear_ratio = 1.0;
                prev_gear_ratio = 1.0;
                prev_gear_lb = default_params.Gearset_n_ub_rpm_6;
                break;
        }
        double new_engine_rpm = prev_gear_lb * curr_gear_ratio / prev_gear_ratio; 
        double engine_rpm_slope = (curr_gear_lb - new_engine_rpm) / (2.0 / TIME_STEP_S); // Aim to shift in 2 seconds
        rtObject.rtU.engine_rpm = new_engine_rpm;
        rtObject.rtU.curr_gear = i;

        for(int j = 0; j < 3/TIME_STEP_S; j++)
        {
            rtObject.rtU.engine_rpm += engine_rpm_slope;
            stepNTimes(rtObject, 1);

            if(rtObject.rtY.gear_out != i)
            {
                break;
            }
        }

        EXPECT_EQ(rtObject.rtY.gear_out, i-1);
        EXPECT_NEAR(rtObject.rtU.engine_rpm, curr_gear_lb, 25);
        stepForTime(rtObject, default_params.Shift_timeout_s / 2.0); // Wait for gear shift to complete
    }

    // If gearbox rejects shift (i.e. we send a command, and it does not respond in Shift_timeout_s),
    // We should revert to sending the command for the previous gear and try again, if necessary,
    // Min_shift_delay_s seconds after the first attempt
    rtObject.rtU.curr_gear = 1;
    rtObject.rtU.engine_rpm = 2000;
    stepForTime(rtObject, 2.0); // reset
    EXPECT_EQ(rtObject.rtU.curr_gear, 1);

    rtObject.rtU.engine_rpm = default_params.Gearset_n_ub_rpm_1 + 100.0;

    while(rtObject.rtY.gear_out == 1) {
        stepNTimes(rtObject, 1);
    }

    EXPECT_EQ(rtObject.rtY.gear_out, 2);

    s_to_shift = 0;

    while(rtObject.rtY.gear_out == 2) {
        stepNTimes(rtObject, 1);
        s_to_shift += TIME_STEP_S;
    }

    EXPECT_EQ(rtObject.rtY.gear_out, 1);
    EXPECT_NEAR(s_to_shift, default_params.Shift_timeout_s, 2*TIME_STEP_S);


}