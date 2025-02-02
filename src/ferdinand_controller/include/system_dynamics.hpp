/* Copyright 2023 Sashank Modali

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License. 
*/

/*
	System Dynamics header
*/

/*
@author: Sashank Modali
@last_major_update: 2023-07-23 22:28:52
*/


#ifndef SYSTEM_DYNAMICS_HPP
#define SYSTEM_DYNAMICS_HPP




#include "rclcpp/rclcpp.hpp"
// #include "unistd.h"// if __unix
// #include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/LU>
// #include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <vector>
                                                                                                                                                                                                                                                                                                                                          
// namespace control                                                                                                                                                                                                                                                                                                                                          
// {

struct vehiclePhysicsParamsType{
    double mass = 720.;
    double moiZ = 1000.;
    double velocityDragCoeff = 0.1;
    double trackWidth = 1.5813;
    double frontAxleMomentArm = 1.723644;
    double rearAxleMomentArm = 1.248156;
    double frontWheelRadius = 0.3;
    double rearWheelRadius = 0.3;
    double frontWheelMOI = 1.35;
    double rearWheelMOI = 1.35;
    double tireStiffnessX = 27.329; //-stiffness*Load/linearizationSpeed*slippingSpeed = tire force.
    double tireStiffnessY = 26.502; //-stiffness*Load/linearizationSpeed*slippingSpeed = tire force.
//   double linearizationSpeed = 20.;
    double steeringAngleGain = M_PI/180.*9.0/19.0;  // 1 degree steering angle = how many radians for wheel turn angle (delta in radians)
    // double brakingTorqueGain = 0.75*0.134*M_PI*0.008*0.008*4.;    // 1 command value = how much braking torque on the wheels
    double brakingTorqueGain = 2465.09319271;    // 1 command value = how much braking torque on the wheels
    double engineRPMUpperLimit = 6000.;
    double engineRPMLowerLimit = 1500.;
    // Engine Data - Hard Coded
    const double engineCurveRPM[17]={0., 1500., 3250., 3500., 3750., 4000., 4250., 4500., 4750., 5000., 
                                    5250., 5500., 5750., 6000., 6250., 6500., 6750.};
    // Output torque converted to N*m
    const double engineCurveTorque[17]={0.0, 108.47, 216.93, 322.68, 399.97, 481.32, 515.21, 508.43, 501.65, 490.81,
                                        485.38, 474.54, 463.69, 454.20, 436.57, 416.24, 393.19}; // FIXME: first value edited! initially 0!
    static const uint8_t nGear = 6;
    // Gear ratio [including differential drive stage (*3)].
    const double gearRatio[nGear] = {8.7501, 5.625, 4.143, 3.345, 2.880, 2.667};

    const double torque_coeffs[11] = {-5.95520059e-06,  2.63075181e-04, -4.73910814e-03,  4.41159914e-02,
    -2.17594115e-01,  4.74959379e-01,  1.35258404e-01, -2.34751174e+00,
    3.27807309e+00, -6.94953662e-01,  2.65404324e-01};
    // -5.95520059e-06,  2.63075181e-04, -4.73910814e-03,  4.41159914e-02, -2.17594115e-01,  4.74959379e-01,  1.35258404e-01, -2.34751174e+00, 3.27807309e+00, -6.94953662e-01,  2.65404324e-01};//{-5.99470011e-06,  2.64534653e-04, -4.75622792e-03,  4.41031032e-02, -2.15432241e-01,  4.52626624e-01,  2.44494587e-01, -2.62029695e+00, 3.55442838e+00, -6.13427634e-01,  1.60201835e-02};//{0.000000021008588, -0.000003663718416, 0.000217794320912, -0.005035315066231, 0.044375985993592, 0.001277251932955};

};

// } // end of namespace

#endif