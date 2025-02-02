//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: acceleration_interface.h
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
#ifndef RTW_HEADER_acceleration_interface_h_
#define RTW_HEADER_acceleration_interface_h_
#include "rtwtypes.h"
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_TNRc1o1NMjCvKeMmZogo6E_
#define DEFINED_TYPEDEF_FOR_struct_TNRc1o1NMjCvKeMmZogo6E_

struct struct_TNRc1o1NMjCvKeMmZogo6E
{
  real_T i_gear_0;
  real_T i_gear_1;
  real_T i_gear_2;
  real_T i_gear_3;
  real_T i_gear_4;
  real_T i_gear_5;
  real_T i_gear_6;
  real_T i_gearRat_0;
  real_T i_gearRat_1;
  real_T i_gearRat_2;
  real_T i_gearRat_3;
  real_T i_gearRat_4;
  real_T i_gearRat_5;
  real_T i_gearRat_6;
  real_T n_lb_rpm_0;
  real_T n_lb_rpm_1;
  real_T n_lb_rpm_2;
  real_T n_lb_rpm_3;
  real_T n_lb_rpm_4;
  real_T n_lb_rpm_5;
  real_T n_lb_rpm_6;
  real_T eta_gear_0;
  real_T eta_gear_1;
  real_T eta_gear_2;
  real_T eta_gear_3;
  real_T eta_gear_4;
  real_T eta_gear_5;
  real_T eta_gear_6;
  real_T n_ub_rpm_0;
  real_T n_ub_rpm_1;
  real_T n_ub_rpm_2;
  real_T n_ub_rpm_3;
  real_T n_ub_rpm_4;
  real_T n_ub_rpm_5;
  real_T n_ub_rpm_6;
};

#endif

// Class declaration for model acceleration_interface
class acceleration_interface final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct D_Work {
    real_T Reshape[35];                // '<S7>/Reshape'
    real_T UnitDelay_DSTATE;           // '<S11>/Unit Delay'
    real_T DiscreteTimeIntegrator_DSTATE;// '<S11>/Discrete-Time Integrator'
    real_T UD_DSTATE;                  // '<S17>/UD'
    real_T UD_DSTATE_p;                // '<S16>/UD'
    real_T UD_DSTATE_d;                // '<S18>/UD'
    real_T desired_gear;               // '<S11>/MATLAB Function'
    struct {
      uint_T desired_gear_not_empty:1; // '<S11>/MATLAB Function'
    } bitsForTID0;

    int8_T DiscreteTimeIntegrator_PrevRese;// '<S11>/Discrete-Time Integrator'
  };

  // External inputs (root inport signals with default storage)
  struct ExternalInputs {
    real32_T curr_velocity_mps;        // '<Root>/curr_velocity_mps'
    real32_T target_accel_mps2;        // '<Root>/target_accel_mps2'
    int8_T curr_gear;                  // '<Root>/curr_gear'
    real32_T engine_rpm;               // '<Root>/engine_rpm'
    boolean_T graceful_stop;           // '<Root>/graceful_stop'
    boolean_T parking_brake;           // '<Root>/parking_brake'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExternalOutputs {
    int8_T gear_out;                   // '<Root>/gear_out'
    real32_T p_brake_kpa;              // '<Root>/p_brake_kpa'
    real32_T throttle_pos;             // '<Root>/throttle_pos'
  };

  // Parameters (default storage)
  struct Parameters {
    struct_TNRc1o1NMjCvKeMmZogo6E Gearset;// Variable: Gearset
                                             //  Referenced by:
                                             //    '<S7>/Constant10'
                                             //    '<S7>/Constant11'
                                             //    '<S7>/Constant14'
                                             //    '<S7>/Constant15'
                                             //    '<S7>/Constant18'
                                             //    '<S7>/Constant19'
                                             //    '<S7>/Constant22'
                                             //    '<S7>/Constant23'
                                             //    '<S7>/Constant26'
                                             //    '<S7>/Constant27'
                                             //    '<S7>/Constant3'
                                             //    '<S7>/Constant35'
                                             //    '<S7>/Constant36'
                                             //    '<S7>/Constant37'
                                             //    '<S7>/Constant38'
                                             //    '<S7>/Constant39'
                                             //    '<S7>/Constant4'
                                             //    '<S7>/Constant40'
                                             //    '<S7>/Constant41'
                                             //    '<S7>/Constant42'
                                             //    '<S7>/Constant43'
                                             //    '<S7>/Constant44'
                                             //    '<S7>/Constant45'
                                             //    '<S7>/Constant46'
                                             //    '<S7>/Constant47'
                                             //    '<S7>/Constant48'
                                             //    '<S7>/Constant49'
                                             //    '<S7>/Constant50'
                                             //    '<S7>/Constant51'
                                             //    '<S7>/Constant52'
                                             //    '<S7>/Constant53'
                                             //    '<S7>/Constant54'
                                             //    '<S7>/Constant55'
                                             //    '<S7>/Constant56'
                                             //    '<S7>/Constant57'

    real_T A_Vehicle_m2;               // Variable: A_Vehicle_m2
                                          //  Referenced by: '<S4>/Constant'

    real_T A_caliper_mm2;              // Variable: A_caliper_mm2
                                          //  Referenced by: '<S4>/Constant1'

    real_T Acc_graceful_stop_ms2;      // Variable: Acc_graceful_stop_ms2
                                          //  Referenced by: '<S4>/Constant13'

    real_T Acc_max_ms2;                // Variable: Acc_max_ms2
                                          //  Referenced by: '<S4>/Constant11'

    real_T Acc_min_ms2;                // Variable: Acc_min_ms2
                                          //  Referenced by: '<S4>/Constant12'

    real_T Brake_max_kpa;              // Variable: Brake_max_kpa
                                          //  Referenced by: '<S4>/Constant10'

    real_T Coef_drag;                  // Variable: Coef_drag
                                          //  Referenced by: '<S4>/Constant8'

    real_T ENgineLimits_Ts;            // Variable: ENgineLimits_Ts
                                          //  Referenced by:
                                          //    '<S16>/Gain'
                                          //    '<S17>/Gain'
                                          //    '<S18>/Gain'

    real_T Efficiency_diff_re;         // Variable: Efficiency_diff_re
                                          //  Referenced by: '<S10>/Constant3'

    real_T M_vehicle_kg;               // Variable: M_vehicle_kg
                                          //  Referenced by: '<S4>/Constant2'

    real_T Min_shift_delay_s;          // Variable: Min_shift_delay_s
                                          //  Referenced by: '<S4>/Constant14'

    real_T Mue_k_brake;                // Variable: Mue_k_brake
                                          //  Referenced by: '<S4>/Constant3'

    real_T R_brake_lever_m;            // Variable: R_brake_lever_m
                                          //  Referenced by: '<S4>/Constant4'

    real_T R_tire_fr;                  // Variable: R_tire_fr
                                          //  Referenced by: '<S4>/Constant5'

    real_T R_tire_re;                  // Variable: R_tire_re
                                          //  Referenced by: '<S4>/Constant6'

    real_T Ratio_diff_re;              // Variable: Ratio_diff_re
                                          //  Referenced by: '<S4>/Constant7'

    real_T Shift_timeout_s;            // Variable: Shift_timeout_s
                                          //  Referenced by: '<S4>/Constant15'

    real_T Throttle_max_p;             // Variable: Throttle_max_p
                                          //  Referenced by: '<S4>/Constant9'

    real_T breakpoints_x[71];          // Variable: breakpoints_x
                                          //  Referenced by:
                                          //    '<S10>/30percTorque'
                                          //    '<S10>/MaxTorque'
                                          //    '<S10>/MinTorque'

    real_T breakpoints_y[8];           // Variable: breakpoints_y
                                          //  Referenced by:
                                          //    '<S10>/30percTorque'
                                          //    '<S10>/MaxTorque'
                                          //    '<S10>/MinTorque'

    real_T roh_air;                    // Variable: roh_air
                                          //  Referenced by: '<S8>/Gain1'

    real_T table2d_map[568];           // Variable: table2d_map
                                          //  Referenced by:
                                          //    '<S10>/30percTorque'
                                          //    '<S10>/MaxTorque'
                                          //    '<S10>/MinTorque'

    real_T ts;                         // Variable: ts
                                          //  Referenced by:
                                          //    '<S16>/Gain'
                                          //    '<S17>/Gain'
                                          //    '<S18>/Gain'

    real_T TransferFcnFirstOrder1_ICPrevOu;
                              // Mask Parameter: TransferFcnFirstOrder1_ICPrevOu
                                 //  Referenced by: '<S17>/UD'

    real_T TransferFcnFirstOrder_ICPrevOut;
                              // Mask Parameter: TransferFcnFirstOrder_ICPrevOut
                                 //  Referenced by: '<S16>/UD'

    real_T TransferFcnFirstOrder2_ICPrevOu;
                              // Mask Parameter: TransferFcnFirstOrder2_ICPrevOu
                                 //  Referenced by: '<S18>/UD'

    real_T Gain_Gain;                  // Expression: -1
                                          //  Referenced by: '<S20>/Gain'

    real_T Gain1_Gain;                 // Expression: 1/4
                                          //  Referenced by: '<S20>/Gain1'

    real_T Gain3_Gain;                 // Expression: 0.5
                                          //  Referenced by: '<S20>/Gain3'

    real_T Gain2_Gain;                 // Expression: 1000
                                          //  Referenced by: '<S20>/Gain2'

    real_T Constant1_Value;            // Expression: 0
                                          //  Referenced by: '<S12>/Constant1'

    real_T Constant_Value;             // Expression: 0
                                          //  Referenced by: '<S12>/Constant'

    real_T Switch_Threshold;           // Expression: 3
                                          //  Referenced by: '<S12>/Switch'

    real_T Saturation_UpperSat;        // Expression: 0
                                          //  Referenced by: '<S12>/Saturation'

    real_T Saturation_LowerSat;        // Expression: -inf
                                          //  Referenced by: '<S12>/Saturation'

    real_T Gain1_Gain_p;               // Expression: -1
                                          //  Referenced by: '<S12>/Gain1'

    real_T Switch1_Threshold;          // Expression: 0
                                          //  Referenced by: '<S12>/Switch1'

    real_T Gain_Gain_n;                // Expression: 100
                                          //  Referenced by: '<S6>/Gain'

    real_T Gain_Gain_k;                // Expression: 1e-6
                                          //  Referenced by: '<S4>/Gain'

    real_T AddConstant_Bias;           // Expression: 1
                                          //  Referenced by: '<Root>/Add Constant'

    real_T UnitDelay_InitialCondition; // Expression: 0
                                          //  Referenced by: '<S11>/Unit Delay'

    real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S11>/Discrete-Time Integrator'

    real_T DiscreteTimeIntegrator_IC;  // Expression: 0
                                          //  Referenced by: '<S11>/Discrete-Time Integrator'

    real_T AddConstant1_Bias;          // Expression: -1
                                          //  Referenced by: '<Root>/Add Constant1'

    real_T Saturation_UpperSat_m;      // Expression: 6
                                          //  Referenced by: '<Root>/Saturation'

    real_T Saturation_LowerSat_h;      // Expression: 0
                                          //  Referenced by: '<Root>/Saturation'

    real_T Gain_Gain_i;                // Expression: 0.5
                                          //  Referenced by: '<S8>/Gain'

    real_T Constant5_Value;            // Expression: 0
                                          //  Referenced by: '<S10>/Constant5'

    real_T Constant4_Value;            // Expression: 1
                                          //  Referenced by: '<S10>/Constant4'

    real_T Constant6_Value;            // Expression: 0.3
                                          //  Referenced by: '<S10>/Constant6'

    real_T Constant_Value_k;           // Expression: 1
                                          //  Referenced by: '<S11>/Constant'

    real32_T Zero_Value;               // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S3>/Zero'

    uint32_T MinTorque_maxIndex[2];    // Computed Parameter: MinTorque_maxIndex
                                          //  Referenced by: '<S10>/MinTorque'

    uint32_T MaxTorque_maxIndex[2];    // Computed Parameter: MaxTorque_maxIndex
                                          //  Referenced by: '<S10>/MaxTorque'

    uint32_T u0percTorque_maxIndex[2];
                                    // Computed Parameter: u0percTorque_maxIndex
                                       //  Referenced by: '<S10>/30percTorque'

  };

  // Real-time Model Data Structure
  struct RT_MODEL {
    const char_T * volatile errorStatus;
  };

  // Copy Constructor
  acceleration_interface(acceleration_interface const&) = delete;

  // Assignment Operator
  acceleration_interface& operator= (acceleration_interface const&) & = delete;

  // Move Constructor
  acceleration_interface(acceleration_interface &&) = delete;

  // Move Assignment Operator
  acceleration_interface& operator= (acceleration_interface &&) = delete;

  // Real-Time Model get method
  acceleration_interface::RT_MODEL * getRTM();

  // External inputs
  ExternalInputs rtU;

  // External outputs
  ExternalOutputs rtY;

  // Tunable parameters
  static Parameters rtP;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  acceleration_interface();

  // Destructor
  ~acceleration_interface();

  // private data and function members
 private:
  // Block states
  D_Work rtDWork;

  // Real-Time Model
  RT_MODEL rtM;
};

#define NOT_USING_NONFINITE_LITERALS   1

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static void rt_InitInfAndNaN(size_t realSize);
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
  struct BigEndianIEEEDouble {
    struct {
      uint32_T wordH;
      uint32_T wordL;
    } words;
  };

  struct LittleEndianIEEEDouble {
    struct {
      uint32_T wordL;
      uint32_T wordH;
    } words;
  };

  struct IEEESingle {
    union {
      real32_T wordLreal;
      uint32_T wordLuint;
    } wordL;
  };
}                                      // extern "C"

extern "C"
{
  static real_T rtGetInf(void);
  static real32_T rtGetInfF(void);
  static real_T rtGetMinusInf(void);
  static real32_T rtGetMinusInfF(void);
}                                      // extern "C"

extern "C"
{
  static real_T rtGetNaN(void);
  static real32_T rtGetNaNF(void);
}                                      // extern "C"

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S16>/Data Type Duplicate' : Unused code path elimination
//  Block '<S17>/Data Type Duplicate' : Unused code path elimination
//  Block '<S18>/Data Type Duplicate' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'acceleration_interface'
//  '<S1>'   : 'acceleration_interface/Angular Velocity Conversion'
//  '<S2>'   : 'acceleration_interface/HandleGracefulStop'
//  '<S3>'   : 'acceleration_interface/HandleParkingBrake'
//  '<S4>'   : 'acceleration_interface/Initialization'
//  '<S5>'   : 'acceleration_interface/Subsystem'
//  '<S6>'   : 'acceleration_interface/Subsystem1'
//  '<S7>'   : 'acceleration_interface/Initialization/Subsystem'
//  '<S8>'   : 'acceleration_interface/Subsystem/Feedforward_aero'
//  '<S9>'   : 'acceleration_interface/Subsystem/Feedforward_long'
//  '<S10>'  : 'acceleration_interface/Subsystem1/Convert2throttle'
//  '<S11>'  : 'acceleration_interface/Subsystem1/GearSelection'
//  '<S12>'  : 'acceleration_interface/Subsystem1/Subsystem'
//  '<S13>'  : 'acceleration_interface/Subsystem1/Convert2throttle/Angular Velocity Conversion'
//  '<S14>'  : 'acceleration_interface/Subsystem1/Convert2throttle/MATLAB Function1'
//  '<S15>'  : 'acceleration_interface/Subsystem1/Convert2throttle/MATLAB Function2'
//  '<S16>'  : 'acceleration_interface/Subsystem1/Convert2throttle/Transfer Fcn First Order'
//  '<S17>'  : 'acceleration_interface/Subsystem1/Convert2throttle/Transfer Fcn First Order1'
//  '<S18>'  : 'acceleration_interface/Subsystem1/Convert2throttle/Transfer Fcn First Order2'
//  '<S19>'  : 'acceleration_interface/Subsystem1/GearSelection/MATLAB Function'
//  '<S20>'  : 'acceleration_interface/Subsystem1/Subsystem/Convert2BrakePressure'

#endif                                 // RTW_HEADER_acceleration_interface_h_

//
// File trailer for generated code.
//
// [EOF]
//
