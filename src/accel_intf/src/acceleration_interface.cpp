//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: acceleration_interface.cpp
//
// Code generated for Simulink model 'acceleration_interface'.
//
// Model version                  : 13.49
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Sat Jan 20 18:20:14 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. RAM efficiency
//    2. Execution efficiency
// Validation result: Not run
//
#include "acceleration_interface.h"
#include <cmath>
#include "rtwtypes.h"
#include <stddef.h>
#define NumBitsPerChar                 8U

static real_T look2_binlx(real_T u0, real_T u1, const real_T bp0[], const real_T
  bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride);
extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

static real_T look2_binlx(real_T u0, real_T u1, const real_T bp0[], const real_T
  bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride)
{
  real_T fractions[2];
  real_T frac;
  real_T yL_0d0;
  real_T yL_0d1;
  uint32_T bpIndices[2];
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  // Column-major Lookup 2-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex[0U]]) {
    // Binary Search
    bpIdx = (maxIndex[0U] >> 1U);
    iLeft = 0U;
    iRght = maxIndex[0U];
    while ((iRght - iLeft) > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = ((iRght + iLeft) >> 1U);
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex[0U] - 1U;
    frac = (u0 - bp0[maxIndex[0U] - 1U]) / (bp0[maxIndex[0U]] - bp0[maxIndex[0U]
      - 1U]);
  }

  fractions[0U] = frac;
  bpIndices[0U] = iLeft;

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u1 <= bp1[0U]) {
    iLeft = 0U;
    frac = (u1 - bp1[0U]) / (bp1[1U] - bp1[0U]);
  } else if (u1 < bp1[maxIndex[1U]]) {
    // Binary Search
    bpIdx = (maxIndex[1U] >> 1U);
    iLeft = 0U;
    iRght = maxIndex[1U];
    while ((iRght - iLeft) > 1U) {
      if (u1 < bp1[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = ((iRght + iLeft) >> 1U);
    }

    frac = (u1 - bp1[iLeft]) / (bp1[iLeft + 1U] - bp1[iLeft]);
  } else {
    iLeft = maxIndex[1U] - 1U;
    frac = (u1 - bp1[maxIndex[1U] - 1U]) / (bp1[maxIndex[1U]] - bp1[maxIndex[1U]
      - 1U]);
  }

  // Column-major Interpolation 2-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'

  bpIdx = (iLeft * stride) + bpIndices[0U];
  yL_0d0 = table[bpIdx];
  yL_0d0 += (table[bpIdx + 1U] - yL_0d0) * fractions[0U];
  bpIdx += stride;
  yL_0d1 = table[bpIdx];
  return (((((table[bpIdx + 1U] - yL_0d1) * fractions[0U]) + yL_0d1) - yL_0d0) *
          frac) + yL_0d0;
}

// Model step function
void acceleration_interface::step()
{
  real_T rtb_Add;
  real_T rtb_AddConstant;
  real_T rtb_MaxTorque;
  real_T rtb_UnitConversion;
  real_T rtb_UnitDelay;
  real_T target_gear;
  int32_T tmp;
  boolean_T shift_needed;

  // Outputs for Atomic SubSystem: '<Root>/Initialization'
  // Reshape: '<S7>/Reshape' incorporates:
  //   Constant: '<S7>/Constant10'
  //   Constant: '<S7>/Constant11'
  //   Constant: '<S7>/Constant14'
  //   Constant: '<S7>/Constant15'
  //   Constant: '<S7>/Constant18'
  //   Constant: '<S7>/Constant19'
  //   Constant: '<S7>/Constant22'
  //   Constant: '<S7>/Constant23'
  //   Constant: '<S7>/Constant26'
  //   Constant: '<S7>/Constant27'
  //   Constant: '<S7>/Constant3'
  //   Constant: '<S7>/Constant35'
  //   Constant: '<S7>/Constant36'
  //   Constant: '<S7>/Constant37'
  //   Constant: '<S7>/Constant38'
  //   Constant: '<S7>/Constant39'
  //   Constant: '<S7>/Constant4'
  //   Constant: '<S7>/Constant40'
  //   Constant: '<S7>/Constant41'
  //   Constant: '<S7>/Constant42'
  //   Constant: '<S7>/Constant43'
  //   Constant: '<S7>/Constant44'
  //   Constant: '<S7>/Constant45'
  //   Constant: '<S7>/Constant46'
  //   Constant: '<S7>/Constant47'
  //   Constant: '<S7>/Constant48'
  //   Constant: '<S7>/Constant49'
  //   Constant: '<S7>/Constant50'
  //   Constant: '<S7>/Constant51'
  //   Constant: '<S7>/Constant52'
  //   Constant: '<S7>/Constant53'
  //   Constant: '<S7>/Constant54'
  //   Constant: '<S7>/Constant55'
  //   Constant: '<S7>/Constant56'
  //   Constant: '<S7>/Constant57'

  rtDWork.Reshape[0] = rtP.Gearset.i_gear_0;
  rtDWork.Reshape[1] = rtP.Gearset.i_gearRat_0;
  rtDWork.Reshape[2] = rtP.Gearset.n_lb_rpm_0;
  rtDWork.Reshape[3] = rtP.Gearset.n_ub_rpm_0;
  rtDWork.Reshape[4] = rtP.Gearset.eta_gear_0;
  rtDWork.Reshape[5] = rtP.Gearset.i_gear_1;
  rtDWork.Reshape[6] = rtP.Gearset.i_gearRat_1;
  rtDWork.Reshape[7] = rtP.Gearset.n_lb_rpm_1;
  rtDWork.Reshape[8] = rtP.Gearset.n_ub_rpm_1;
  rtDWork.Reshape[9] = rtP.Gearset.eta_gear_1;
  rtDWork.Reshape[10] = rtP.Gearset.i_gear_2;
  rtDWork.Reshape[11] = rtP.Gearset.i_gearRat_2;
  rtDWork.Reshape[12] = rtP.Gearset.n_lb_rpm_2;
  rtDWork.Reshape[13] = rtP.Gearset.n_ub_rpm_2;
  rtDWork.Reshape[14] = rtP.Gearset.eta_gear_2;
  rtDWork.Reshape[15] = rtP.Gearset.i_gear_3;
  rtDWork.Reshape[16] = rtP.Gearset.i_gearRat_3;
  rtDWork.Reshape[17] = rtP.Gearset.n_lb_rpm_3;
  rtDWork.Reshape[18] = rtP.Gearset.n_ub_rpm_3;
  rtDWork.Reshape[19] = rtP.Gearset.eta_gear_3;
  rtDWork.Reshape[20] = rtP.Gearset.i_gear_4;
  rtDWork.Reshape[21] = rtP.Gearset.i_gearRat_4;
  rtDWork.Reshape[22] = rtP.Gearset.n_lb_rpm_4;
  rtDWork.Reshape[23] = rtP.Gearset.n_ub_rpm_4;
  rtDWork.Reshape[24] = rtP.Gearset.eta_gear_4;
  rtDWork.Reshape[25] = rtP.Gearset.i_gear_5;
  rtDWork.Reshape[26] = rtP.Gearset.i_gearRat_5;
  rtDWork.Reshape[27] = rtP.Gearset.n_lb_rpm_5;
  rtDWork.Reshape[28] = rtP.Gearset.n_ub_rpm_5;
  rtDWork.Reshape[29] = rtP.Gearset.eta_gear_5;
  rtDWork.Reshape[30] = rtP.Gearset.i_gear_6;
  rtDWork.Reshape[31] = rtP.Gearset.i_gearRat_6;
  rtDWork.Reshape[32] = rtP.Gearset.n_lb_rpm_6;
  rtDWork.Reshape[33] = rtP.Gearset.n_ub_rpm_6;
  rtDWork.Reshape[34] = rtP.Gearset.eta_gear_6;

  // End of Outputs for SubSystem: '<Root>/Initialization'

  // Bias: '<Root>/Add Constant' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion6'
  //   Inport: '<Root>/curr_gear'

  rtb_AddConstant = (static_cast<real_T>(rtU.curr_gear)) + rtP.AddConstant_Bias;

  // UnitConversion: '<S1>/Unit Conversion' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion5'
  //   Inport: '<Root>/engine_rpm'

  // Unit Conversion - from: rpm to: rad/s
  // Expression: output = (0.10472*input) + (0)
  rtb_UnitConversion = 0.10471975511965977 * rtU.engine_rpm;

  // UnitDelay: '<S11>/Unit Delay'
  rtb_UnitDelay = rtDWork.UnitDelay_DSTATE;

  // DiscreteIntegrator: '<S11>/Discrete-Time Integrator' incorporates:
  //   UnitDelay: '<S11>/Unit Delay'

  if ((rtDWork.UnitDelay_DSTATE <= 0.0) &&
      (rtDWork.DiscreteTimeIntegrator_PrevRese == 1)) {
    rtDWork.DiscreteTimeIntegrator_DSTATE = rtP.DiscreteTimeIntegrator_IC;
  }

  // MATLAB Function: '<S11>/MATLAB Function' incorporates:
  //   Constant: '<S4>/Constant14'
  //   Constant: '<S4>/Constant15'
  //   DiscreteIntegrator: '<S11>/Discrete-Time Integrator'
  //   MATLAB Function: '<S10>/MATLAB Function1'
  //   Reshape: '<S7>/Reshape'
  //   UnitDelay: '<S11>/Unit Delay'

  rtDWork.UnitDelay_DSTATE = 1.0;
  if (!rtDWork.bitsForTID0.desired_gear_not_empty) {
    rtDWork.desired_gear = rtb_AddConstant;
    rtDWork.bitsForTID0.desired_gear_not_empty = true;
  }

  shift_needed = false;
  tmp = ((static_cast<int32_T>(rtb_AddConstant)) - 1) * 5;
  if ((rtb_UnitConversion > (rtDWork.Reshape[tmp + 3] * 0.10471975511965977)) &&
      (rtb_AddConstant < 7.0)) {
    shift_needed = true;
    target_gear = rtb_AddConstant + 1.0;
  } else if ((rtb_UnitConversion < (rtDWork.Reshape[tmp + 2] *
               0.10471975511965977)) && (rtb_AddConstant > 1.0)) {
    shift_needed = true;
    target_gear = rtb_AddConstant - 1.0;
  } else {
    target_gear = rtb_AddConstant;
  }

  if ((rtb_AddConstant != rtDWork.desired_gear) &&
      (rtDWork.DiscreteTimeIntegrator_DSTATE >= rtP.Shift_timeout_s)) {
    rtDWork.desired_gear = rtb_AddConstant;
  }

  if (shift_needed && (rtDWork.DiscreteTimeIntegrator_DSTATE >=
                       rtP.Min_shift_delay_s)) {
    rtDWork.desired_gear = target_gear;
    rtDWork.UnitDelay_DSTATE = 0.0;
  }

  // Bias: '<Root>/Add Constant1' incorporates:
  //   MATLAB Function: '<S11>/MATLAB Function'

  target_gear = rtDWork.desired_gear + rtP.AddConstant1_Bias;

  // Saturate: '<Root>/Saturation'
  if (target_gear > rtP.Saturation_UpperSat_m) {
    target_gear = rtP.Saturation_UpperSat_m;
  } else if (target_gear < rtP.Saturation_LowerSat_h) {
    target_gear = rtP.Saturation_LowerSat_h;
  }

  // Outport: '<Root>/gear_out' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion4'
  //   Saturate: '<Root>/Saturation'

  rtY.gear_out = static_cast<int8_T>(std::floor(target_gear));

  // Switch: '<S2>/Switch' incorporates:
  //   Constant: '<S4>/Constant13'
  //   Inport: '<Root>/graceful_stop'
  //   Inport: '<Root>/target_accel_mps2'
  //   Logic: '<S2>/NOT'

  if (!rtU.graceful_stop) {
    rtb_MaxTorque = rtU.target_accel_mps2;
  } else {
    rtb_MaxTorque = rtP.Acc_graceful_stop_ms2;
  }

  // Sum: '<S5>/Add' incorporates:
  //   Constant: '<S4>/Constant'
  //   Constant: '<S4>/Constant11'
  //   Constant: '<S4>/Constant12'
  //   Constant: '<S4>/Constant2'
  //   Constant: '<S4>/Constant8'
  //   DataTypeConversion: '<Root>/Data Type Conversion'
  //   Gain: '<S8>/Gain'
  //   Gain: '<S8>/Gain1'
  //   Inport: '<Root>/curr_velocity_mps'
  //   MinMax: '<S5>/Max'
  //   MinMax: '<S5>/Min'
  //   Product: '<S8>/Product'
  //   Product: '<S8>/Product1'
  //   Product: '<S8>/Product2'
  //   Product: '<S9>/Product'
  //   Switch: '<S2>/Switch'

  rtb_Add = ((((((static_cast<real_T>(rtU.curr_velocity_mps)) *
                 rtU.curr_velocity_mps) * rtP.Gain_Gain_i) * rtP.roh_air) *
              rtP.A_Vehicle_m2) * rtP.Coef_drag) + (std::fmax(std::fmin
    (rtb_MaxTorque, rtP.Acc_max_ms2), rtP.Acc_min_ms2) * rtP.M_vehicle_kg);

  // UnitConversion: '<S13>/Unit Conversion'
  // Unit Conversion - from: rad/s to: rpm
  // Expression: output = (9.5493*input) + (0)
  rtb_UnitConversion *= 9.5492965855137211;

  // Lookup_n-D: '<S10>/MinTorque' incorporates:
  //   Constant: '<S10>/Constant5'
  //   Lookup_n-D: '<S10>/30percTorque'

  rtb_MaxTorque = look2_binlx(rtb_UnitConversion, rtP.Constant5_Value,
    rtP.breakpoints_x, rtP.breakpoints_y, rtP.table2d_map,
    rtP.MinTorque_maxIndex, 71U);

  // Gain: '<S17>/Gain' incorporates:
  //   Gain: '<S16>/Gain'
  //   Gain: '<S18>/Gain'

  target_gear = std::exp((-rtP.ts) / rtP.ENgineLimits_Ts);

  // Sum: '<S17>/Sum' incorporates:
  //   Gain: '<S17>/Gain'
  //   Sum: '<S17>/Diff'
  //   UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/Sum':
  //
  //   Add in CPU
  //
  //  Block description for '<S17>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  rtDWork.UD_DSTATE = ((rtDWork.UD_DSTATE - rtb_MaxTorque) * target_gear) +
    rtb_MaxTorque;

  // MATLAB Function: '<S10>/MATLAB Function1' incorporates:
  //   Constant: '<S10>/Constant3'
  //   Constant: '<S4>/Constant6'
  //   Constant: '<S4>/Constant7'
  //   Constant: '<S7>/Constant35'
  //   Constant: '<S7>/Constant51'
  //   Reshape: '<S7>/Reshape'
  //   SignalConversion generated from: '<S14>/ SFunction '

  if (rtb_AddConstant != 1.0) {
    rtb_MaxTorque = (rtDWork.Reshape[tmp + 1] * (rtP.Ratio_diff_re *
      rtP.Efficiency_diff_re)) * rtDWork.Reshape[tmp + 4];
    rtb_AddConstant = (rtb_Add * rtP.R_tire_re) / rtb_MaxTorque;
    rtb_MaxTorque /= rtP.R_tire_re;
  } else {
    // Outputs for Atomic SubSystem: '<Root>/Initialization'
    rtb_AddConstant = (rtb_Add * rtP.R_tire_re) / (((rtP.Ratio_diff_re *
      rtP.Efficiency_diff_re) * rtP.Gearset.i_gearRat_2) *
      rtP.Gearset.eta_gear_0);

    // End of Outputs for SubSystem: '<Root>/Initialization'
    rtb_MaxTorque = 0.0;
  }

  // Logic: '<S3>/NOT' incorporates:
  //   Inport: '<Root>/parking_brake'

  shift_needed = !rtU.parking_brake;

  // Switch: '<S3>/Switch1'
  if (shift_needed) {
    // Switch: '<S12>/Switch' incorporates:
    //   Constant: '<S12>/Constant'
    //   DataTypeConversion: '<Root>/Data Type Conversion'
    //   Inport: '<Root>/curr_velocity_mps'
    //   Product: '<S10>/Product'
    //   UnitDelay: '<S17>/UD'
    //
    //  Block description for '<S17>/UD':
    //
    //   Store in Global RAM

    if (rtU.curr_velocity_mps > rtP.Switch_Threshold) {
      rtb_MaxTorque *= rtDWork.UD_DSTATE;
    } else {
      rtb_MaxTorque = rtP.Constant_Value;
    }

    // Sum: '<S12>/Add' incorporates:
    //   Switch: '<S12>/Switch'

    rtb_Add -= rtb_MaxTorque;

    // Saturate: '<S12>/Saturation'
    if (rtb_Add > rtP.Saturation_UpperSat) {
      rtb_Add = rtP.Saturation_UpperSat;
    } else if (rtb_Add < rtP.Saturation_LowerSat) {
      rtb_Add = rtP.Saturation_LowerSat;
    }

    // End of Saturate: '<S12>/Saturation'

    // Switch: '<S12>/Switch1' incorporates:
    //   Constant: '<S12>/Constant1'
    //   Constant: '<S4>/Constant1'
    //   Constant: '<S4>/Constant3'
    //   Constant: '<S4>/Constant4'
    //   Constant: '<S4>/Constant5'
    //   Constant: '<S4>/Constant6'
    //   Gain: '<S12>/Gain1'
    //   Gain: '<S20>/Gain'
    //   Gain: '<S20>/Gain1'
    //   Gain: '<S20>/Gain2'
    //   Gain: '<S20>/Gain3'
    //   Gain: '<S4>/Gain'
    //   Product: '<S20>/Divide1'
    //   Product: '<S20>/Divide2'
    //   Product: '<S20>/Divide3'
    //   Product: '<S20>/Product'
    //   Sum: '<S20>/Add'

    if ((rtP.Gain1_Gain_p * rtb_Add) > rtP.Switch1_Threshold) {
      // Outputs for Atomic SubSystem: '<Root>/Initialization'
      rtb_Add = (1.0 / ((((rtP.Gain_Gain_k * rtP.A_caliper_mm2) *
                          rtP.Mue_k_brake) * rtP.R_brake_lever_m) *
                        rtP.Gain2_Gain)) * (((rtP.R_tire_fr + rtP.R_tire_re) *
        rtP.Gain3_Gain) * ((rtP.Gain_Gain * rtb_Add) * rtP.Gain1_Gain));

      // End of Outputs for SubSystem: '<Root>/Initialization'
    } else {
      rtb_Add = rtP.Constant1_Value;
    }

    // End of Switch: '<S12>/Switch1'

    // Outport: '<Root>/p_brake_kpa' incorporates:
    //   Constant: '<S4>/Constant10'
    //   DataTypeConversion: '<Root>/Data Type Conversion3'
    //   MinMax: '<S6>/Max1'

    rtY.p_brake_kpa = static_cast<real32_T>(std::fmin(rtP.Brake_max_kpa, rtb_Add));
  } else {
    // Outport: '<Root>/p_brake_kpa' incorporates:
    //   Constant: '<S4>/Constant10'

    rtY.p_brake_kpa = static_cast<real32_T>(rtP.Brake_max_kpa);
  }

  // End of Switch: '<S3>/Switch1'

  // Lookup_n-D: '<S10>/MaxTorque' incorporates:
  //   Constant: '<S10>/Constant4'
  //   Lookup_n-D: '<S10>/30percTorque'

  rtb_MaxTorque = look2_binlx(rtb_UnitConversion, rtP.Constant4_Value,
    rtP.breakpoints_x, rtP.breakpoints_y, rtP.table2d_map,
    rtP.MaxTorque_maxIndex, 71U);

  // Sum: '<S16>/Sum' incorporates:
  //   Gain: '<S16>/Gain'
  //   Sum: '<S16>/Diff'
  //   UnitDelay: '<S16>/UD'
  //
  //  Block description for '<S16>/Sum':
  //
  //   Add in CPU
  //
  //  Block description for '<S16>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S16>/UD':
  //
  //   Store in Global RAM

  rtDWork.UD_DSTATE_p = ((rtDWork.UD_DSTATE_p - rtb_MaxTorque) * target_gear) +
    rtb_MaxTorque;

  // Lookup_n-D: '<S10>/30percTorque' incorporates:
  //   Constant: '<S10>/Constant6'

  rtb_UnitConversion = look2_binlx(rtb_UnitConversion, rtP.Constant6_Value,
    rtP.breakpoints_x, rtP.breakpoints_y, rtP.table2d_map,
    rtP.u0percTorque_maxIndex, 71U);

  // Sum: '<S18>/Sum' incorporates:
  //   Gain: '<S18>/Gain'
  //   Sum: '<S18>/Diff'
  //   UnitDelay: '<S18>/UD'
  //
  //  Block description for '<S18>/Sum':
  //
  //   Add in CPU
  //
  //  Block description for '<S18>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S18>/UD':
  //
  //   Store in Global RAM

  rtDWork.UD_DSTATE_d = ((rtDWork.UD_DSTATE_d - rtb_UnitConversion) *
    target_gear) + rtb_UnitConversion;

  // MATLAB Function: '<S10>/MATLAB Function2' incorporates:
  //   UnitDelay: '<S16>/UD'
  //   UnitDelay: '<S17>/UD'
  //   UnitDelay: '<S18>/UD'
  //
  //  Block description for '<S16>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S18>/UD':
  //
  //   Store in Global RAM

  if (rtb_AddConstant > rtDWork.UD_DSTATE_d) {
    rtb_UnitConversion = (((rtb_AddConstant - rtDWork.UD_DSTATE_d) /
      (rtDWork.UD_DSTATE_p - rtDWork.UD_DSTATE_d)) * 0.7) + 0.3;
  } else {
    rtb_UnitConversion = ((rtb_AddConstant - rtDWork.UD_DSTATE) /
                          (rtDWork.UD_DSTATE_d - rtDWork.UD_DSTATE)) * 0.3;
  }

  // Switch: '<S3>/Switch'
  if (shift_needed) {
    // Outport: '<Root>/throttle_pos' incorporates:
    //   Constant: '<S4>/Constant9'
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Gain: '<S6>/Gain'
    //   MATLAB Function: '<S10>/MATLAB Function2'
    //   MinMax: '<S6>/Max'

    rtY.throttle_pos = static_cast<real32_T>(std::fmin(rtP.Gain_Gain_n * std::
      fmax(0.0, std::fmin(rtb_UnitConversion, 1.0)), rtP.Throttle_max_p));
  } else {
    // Outport: '<Root>/throttle_pos' incorporates:
    //   Constant: '<S3>/Zero'

    rtY.throttle_pos = rtP.Zero_Value;
  }

  // End of Switch: '<S3>/Switch'

  // Update for DiscreteIntegrator: '<S11>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S11>/Constant'

  rtDWork.DiscreteTimeIntegrator_DSTATE += rtP.DiscreteTimeIntegrator_gainval *
    rtP.Constant_Value_k;
  if (rtb_UnitDelay > 0.0) {
    rtDWork.DiscreteTimeIntegrator_PrevRese = 1;
  } else if (rtb_UnitDelay < 0.0) {
    rtDWork.DiscreteTimeIntegrator_PrevRese = -1;
  } else if (rtb_UnitDelay == 0.0) {
    rtDWork.DiscreteTimeIntegrator_PrevRese = 0;
  } else {
    rtDWork.DiscreteTimeIntegrator_PrevRese = 2;
  }

  // End of Update for DiscreteIntegrator: '<S11>/Discrete-Time Integrator'
}

// Model initialize function
void acceleration_interface::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // non-finite (run-time) assignments
  rtP.Saturation_LowerSat = rtMinusInf;

  // InitializeConditions for UnitDelay: '<S11>/Unit Delay'
  rtDWork.UnitDelay_DSTATE = rtP.UnitDelay_InitialCondition;

  // InitializeConditions for DiscreteIntegrator: '<S11>/Discrete-Time Integrator' 
  rtDWork.DiscreteTimeIntegrator_DSTATE = rtP.DiscreteTimeIntegrator_IC;
  rtDWork.DiscreteTimeIntegrator_PrevRese = 2;

  // InitializeConditions for UnitDelay: '<S17>/UD'
  //
  //  Block description for '<S17>/UD':
  //
  //   Store in Global RAM

  rtDWork.UD_DSTATE = rtP.TransferFcnFirstOrder1_ICPrevOu;

  // InitializeConditions for UnitDelay: '<S16>/UD'
  //
  //  Block description for '<S16>/UD':
  //
  //   Store in Global RAM

  rtDWork.UD_DSTATE_p = rtP.TransferFcnFirstOrder_ICPrevOut;

  // InitializeConditions for UnitDelay: '<S18>/UD'
  //
  //  Block description for '<S18>/UD':
  //
  //   Store in Global RAM

  rtDWork.UD_DSTATE_d = rtP.TransferFcnFirstOrder2_ICPrevOu;
}

// Constructor
acceleration_interface::acceleration_interface() :
  rtU(),
  rtY(),
  rtDWork(),
  rtM()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
acceleration_interface::~acceleration_interface() = default;

// Real-Time Model get method
acceleration_interface::RT_MODEL * acceleration_interface::getRTM()
{
  return (&rtM);
}

extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result{ (boolean_T) 0 };

    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

extern "C"
{
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T inf{ 0.0 };

    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T minf{ 0.0 };

    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal{ sizeof(real_T) * (NumBitsPerChar) };

    real_T nan{ 0.0 };

    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF{ { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
