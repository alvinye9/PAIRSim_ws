/**
 * @file testing_helpers.hpp
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
#ifndef TESTING_HELPERS_HPP_
#define TESTING_HELPERS_HPP_

#include "acceleration_interface.h"
#define TIME_STEP_S 0.01
#define ENGINE_DYNAMICS_TS 1.0

typedef struct _Parameters {
    double A_vehicle_m2;
    double A_caliper_mm2;
    double Coef_drag;
    double M_vehicle_kg;
    double Mue_k_brake;
    double R_brake_lever_m;
    double R_tire_fr;
    double R_tire_re;
    double Ratio_diff_re;
    double roh_air;
    double Acc_graceful_stop_ms2;
    double Min_shift_delay_s;
    double Shift_timeout_s;
    double Gearset_n_lb_rpm_1;
    double Gearset_n_ub_rpm_1;
    double Gearset_n_lb_rpm_2;
    double Gearset_n_ub_rpm_2;
    double Gearset_n_lb_rpm_3;
    double Gearset_n_ub_rpm_3;
    double Gearset_n_lb_rpm_4;
    double Gearset_n_ub_rpm_4;
    double Gearset_n_lb_rpm_5;
    double Gearset_n_ub_rpm_5;
    double Gearset_n_lb_rpm_6;
    double Gearset_n_ub_rpm_6;

    double max_throttle_pct;
    double max_brake_kpa;
    double max_accel_mps2;
    double max_decel_mps2;
} Parameters;

const Parameters default_params = {
    1.0,  // A_vehicle_m2
    2000.0,  // A_caliper_mm2
    0.6,  // Coef_drag
    787.0,  // M_vehicle_kg
    0.35,  // Mue_k_brake
    0.134,  // R_brake_lever_m
    0.3,  // R_tire_fr
    0.31,  // R_tire_re
    3.0,  // Ratio_diff_re
    1.22,  // roh_air
    -5.0,  // Acc_graceful_stop_ms2
    1.2,  // Min_shift_delay_s
    0.7,  // Shift_timeout_s

    0.0,  // Gearset_n_lb_rpm_1
    4800.0,  // Gearset_n_ub_rpm_1
    2500.0,  // Gearset_n_lb_rpm_2
    6900.0,  // Gearset_n_ub_rpm_2
    4500.0,  // Gearset_n_lb_rpm_3
    6800.0,  // Gearset_n_ub_rpm_3
    5000.0,  // Gearset_n_lb_rpm_4
    6800.0,  // Gearset_n_ub_rpm_4
    5300.0,  // Gearset_n_lb_rpm_5
    6500.0,  // Gearset_n_ub_rpm_5
    5500.0,  // Gearset_n_lb_rpm_6
    6950.0,  // Gearset_n_ub_rpm_6

    100.0,  // max_throttle_pct
    4000.0,  // max_brake_kpa
    10.0,  // max_accel_mps2
    -5.0  // max_decel_mps2
};

inline void setParams(acceleration_interface& rtObject_, Parameters params) {
    rtObject_.rtP.A_Vehicle_m2 = params.A_vehicle_m2;
    rtObject_.rtP.A_caliper_mm2 = params.A_caliper_mm2;
    rtObject_.rtP.Coef_drag = params.Coef_drag;
    rtObject_.rtP.M_vehicle_kg = params.M_vehicle_kg;
    rtObject_.rtP.Mue_k_brake = params.Mue_k_brake;
    rtObject_.rtP.R_brake_lever_m = params.R_brake_lever_m;
    rtObject_.rtP.R_tire_fr = params.R_tire_fr;
    rtObject_.rtP.R_tire_re = params.R_tire_re;
    rtObject_.rtP.Ratio_diff_re = params.Ratio_diff_re;
    rtObject_.rtP.roh_air = params.roh_air;
    rtObject_.rtP.Acc_graceful_stop_ms2 = params.Acc_graceful_stop_ms2;
    rtObject_.rtP.Min_shift_delay_s = params.Min_shift_delay_s;
    rtObject_.rtP.Shift_timeout_s = params.Shift_timeout_s;
    rtObject_.rtP.Gearset.n_lb_rpm_1 = params.Gearset_n_lb_rpm_1;
    rtObject_.rtP.Gearset.n_ub_rpm_1 = params.Gearset_n_ub_rpm_1;
    rtObject_.rtP.Gearset.n_lb_rpm_2 = params.Gearset_n_lb_rpm_2;
    rtObject_.rtP.Gearset.n_ub_rpm_2 = params.Gearset_n_ub_rpm_2;
    rtObject_.rtP.Gearset.n_lb_rpm_3 = params.Gearset_n_lb_rpm_3;
    rtObject_.rtP.Gearset.n_ub_rpm_3 = params.Gearset_n_ub_rpm_3;
    rtObject_.rtP.Gearset.n_lb_rpm_4 = params.Gearset_n_lb_rpm_4;
    rtObject_.rtP.Gearset.n_ub_rpm_4 = params.Gearset_n_ub_rpm_4;
    rtObject_.rtP.Gearset.n_lb_rpm_5 = params.Gearset_n_lb_rpm_5;
    rtObject_.rtP.Gearset.n_ub_rpm_5 = params.Gearset_n_ub_rpm_5;
    rtObject_.rtP.Gearset.n_lb_rpm_6 = params.Gearset_n_lb_rpm_6;
    rtObject_.rtP.Gearset.n_ub_rpm_6 = params.Gearset_n_ub_rpm_6;
    rtObject_.rtP.Throttle_max_p = params.max_throttle_pct;
    rtObject_.rtP.Brake_max_kpa = params.max_brake_kpa;
    rtObject_.rtP.Acc_max_ms2 = params.max_accel_mps2;
    rtObject_.rtP.Acc_min_ms2 = params.max_decel_mps2;
}

inline double calculateAccelerationFromBrake(double brake_kpa, Parameters params) {
    double brake_force = (brake_kpa*1000) * (params.A_caliper_mm2 * 1e-6) * params.Mue_k_brake;
    double wheel_torque = brake_force * params.R_brake_lever_m;
    double effective_radius = (params.R_tire_re + params.R_tire_fr) / 2.0;
    double wheel_force = wheel_torque / effective_radius;
    double accel = 4 * wheel_force / params.M_vehicle_kg;

    return -accel;
}

inline double calculateAccelerationFromDrag(double velocity_mps, Parameters params) {
    // F_drag_N = 0.5 * (air_density_kgpm3) * (drag_coeff_untless) * (area_m2) * (velocity_mps^2)
    // Acceleration = (F_drag_N) / (vehicle_mass_kg)

    double drag_force = 0.5 * params.roh_air * params.Coef_drag * params.A_vehicle_m2 * velocity_mps * velocity_mps;
    double accel = drag_force / params.M_vehicle_kg;

    return -accel;
}

inline void initializeAccelerationInterface(acceleration_interface& rtObject, Parameters params) {
    rtObject.initialize();
    setParams(rtObject, params);
}

inline void stepNTimes(acceleration_interface& rtObject, int n) {
    for (int i = 0; i < n; i++) {
        rtObject.step();
    }
}

inline void stepForTime(acceleration_interface& rtObject, double time_s) {
    int n = (int) (time_s / TIME_STEP_S);
    stepNTimes(rtObject, n);
}

inline void stepTargetToSteadyState(acceleration_interface& rtObject) {
    stepForTime(rtObject, ENGINE_DYNAMICS_TS);
}



#endif // TESTING_HELPERS_HPP