/*
  Copyright 2024 Alec Pannunzio, Andres Hoyos Moreno
  Copyright 2023 Andres Hoyos Moreno
  Copyright 2021 Will Bryan
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions a  nd
   limitations under the License.
*/

#include <ferdinand_controller.hpp>

namespace control {
void StanleyADRC::controlCallback() {
  rclcpp::Time control_time = rclcpp::Clock().now();
  rclcpp::Duration time_diff = control_time - this->recv_time_;
  double age = static_cast<double>(time_diff.seconds()) +
               static_cast<double>(time_diff.nanoseconds()) * 1e-9;

  if (age < 10 * dt) {
    //calculateFFW();
    calculateFB(dt);
    calculateSteeringCmd();
  } else {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Have not received new path in > 0.5s !");
    setCmdsToZeros();
  }

  publishSteering();
  publishDebugSignals();
}

void StanleyADRC::calculateFFW() {
  // Desired yaw rate from feedforward
  double lf = this->get_parameter("vehicle.cg_to_front").as_double();
  double lr = this->get_parameter("vehicle.cg_to_rear").as_double();
  // double Caf = this->get_parameter("vehicle.tire_stiffness_front").as_double();
  // double Car = this->get_parameter("vehicle.tire_stiffness_rear").as_double();
  // double m = this->get_parameter("vehicle.mass").as_double();
  // double Izz = this->get_parameter("vehicle.inertia").as_double();
  //double K3 = this->get_parameter("K3").as_double();

  // double mf = m*lf / (lf + lr);
  // double mr = m*lr / (lf + lr);

  this->feedforward_ = 0.0;
  RCLCPP_DEBUG(this->get_logger(), "Ffw: '%f'", this->feedforward_);
}

void StanleyADRC::receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
  this->current_gear_ = msg->current_gear;
  this->engine_speed_ = msg->engine_rpm;
  this->engine_running_ = msg->engine_on_status;
  RCLCPP_DEBUG(this->get_logger(), "Engine RPM: '%f'", this->engine_speed_);

}

void StanleyADRC::receiveDesiredVelocity(const std_msgs::msg::Float32::SharedPtr msg) {
  this->desired_velocity = msg->data;
}

double StanleyADRC::interpolateEngineMaxTorque(double rpm){

  RCLCPP_DEBUG(this->get_logger(), "rpm %f", rpm);
  size_t size = ((&(this->VP_.engineCurveRPM))[1]-this->VP_.engineCurveRPM)/sizeof(this->VP_.engineCurveRPM[0]);

  int i = 0;
  if (rpm >= this->VP_.engineCurveRPM[size-2]){
    i = (int)size - 2;
  }
  else{
    while (rpm > this->VP_.engineCurveRPM[i+1]) i++;
  }
  double xL = this->VP_.engineCurveRPM[i];
  double yL = this->VP_.engineCurveTorque[i];
  double xR = this->VP_.engineCurveRPM[i+1];
  double yR = this->VP_.engineCurveTorque[i+1];

  if (rpm<xL)
    return yL;
  else if (rpm>xR)
    return yR;
  else{
    double slope = 1; //TODO what is a proper default value for the slope?
    try {
      slope = (yR-yL)/(xR-xL);
    }catch(int exception){
      RCLCPP_ERROR(this->get_logger(), "an exception has occured in interpolateEngineMaxTorque %d", exception);
    }
    // double interpolation_result = yL + slope*(rpm-xL);;
    return yL + slope*(rpm-xL);
  }
}

void StanleyADRC::calculateThrottleBrakeFromTractionForce(double Ft){
  
  double required_traction_torque = Ft * VP_.rearWheelRadius;

  std::stringstream ss;

  // TODO: Calculate required gear:
  double torqueDemandedFromEngine[this->VP_.nGear];
  //double rpmAtThisGear[this->VP_.nGear];
  double torqueMaxAtThisGear[this->VP_.nGear];
  double torqueMinAtThisGear[this->VP_.nGear];

  // Get engine min-max toADRC_lat_control_noderques:
  double torqueEngineMax = interpolateEngineMaxTorque(VP_.engineRPMUpperLimit);
  double torqueEngineMin = interpolateEngineMaxTorque(VP_.engineRPMLowerLimit); 

  if (Ft < 0.0){
    this->brake_cmd = Ft / this->VP_.brakingTorqueGain;
    this->throttle_cmd = 0.0;
  }
  else {
    double accPedalAtThisGear[this->VP_.nGear];
    const double radPerSec2RPM = 30./M_PI;
    int candidateGear = this->VP_.nGear-1;
    double minEffort = 1.;
    bool changedflag = false;

    RCLCPP_DEBUG(this->get_logger(), "Required traction torque: %f", required_traction_torque);

    try {
      for (int i=this->VP_.nGear-1; i>=0; i--){

        torqueDemandedFromEngine[i] = required_traction_torque / this->VP_.gearRatio[i]; // From wheels to engine
        accPedalAtThisGear[i] = torqueDemandedFromEngine[i] / torqueEngineMax; 

        if (accPedalAtThisGear[i] < minEffort){
          if (torqueDemandedFromEngine[i] > torqueEngineMin){
            if (torqueDemandedFromEngine[i] < torqueEngineMax){
              candidateGear = i;
              minEffort = accPedalAtThisGear[i];
              changedflag=true;
              RCLCPP_WARN(this->get_logger(), "MIN EFFORT REACHED %lf AT CANDIDATE GEAR %d", minEffort , candidateGear);
            }
            else{
              break;
            }
          }
          else if (i==0 && changedflag==false) {
            if (this->desired_velocity >= this->speed_) {
              ss.str("");
              ss<< "SINCE ENGINE RPM IS LOWER THAN ACCEPTABLE, but trying to go faster in first gear, SENDING FULL THROTTLE AND ZERO BRAKE" << std::endl;
              RCLCPP_WARN(this->get_logger(), ss.str().c_str());
              //u(1) = 0.;
              candidateGear=0;
            }
            else{
              ss.str("");
              ss<< "SINCE ENGINE RPM IS LOWER THAN ACCEPTABLE, but trying to slow down further in first gear, SENDING ZERO THROTTLE AND FULL BRAKE" << std::endl;
              RCLCPP_WARN(this->get_logger(), ss.str().c_str());
              minEffort = 0.;
              //u(1) = this->VP_.brakingTorqueGain;
            }
          }
        }
      }
      }catch (...) {
        // RCLCPP_WARN(this->get_logger(),"An exception occured with min effort throttle and gear calculations. NAN DETECTED!!!!!!!!");
        ss.str("");
        ss<< "An exception occured with min effort throttle and gear calculations. NAN DETECTED!!!!!!!!" << std::endl;
        RCLCPP_WARN(this->get_logger(), ss.str().c_str());
        // std::cout << "An exception occured: " << e << std::endl;
      }

      this->throttle_cmd = minEffort*100.0;
      this->brake_cmd = 0.0;
      this->gear_cmd = candidateGear + 1;

      RCLCPP_DEBUG(this->get_logger(), "Throttle: '%f'", this->throttle_cmd);
      RCLCPP_DEBUG(this->get_logger(), "Gear: '%i'", this->gear_cmd);

  }

}

std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>> StanleyADRC::KalmanExtendedObserverRamp(Eigen::Matrix<double, 7, 1> x_hat_prev_local, 
                                                                    Eigen::Matrix<double, 7, 7> Pk_prev_local) 
{

  double delta = this->steer;
  double T_traction = this->F_traction / VP_.rearWheelRadius;
  double dy = this->lat_speed;
  double yaw = this->heading;
  double dyaw = this->yaw_rate;
  double eps = 0.01;
  double dx = this->speed_ + eps; // FIXME: Add epsilon?

  double A11 = (Caf*sin(delta)*(dy + dyaw*lf))/(pow(dx,2)*m*(pow((dy + dyaw*lf), 2)/pow(dx,2) + 1));
  double A12 = (dyaw*m - (Caf*sin(delta))/(dx*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  double A13 = 0.0;
  double A14 = (dy*m - (Caf*lf*sin(delta))/(dx*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;

  double A21 = -((Car*(dy - dyaw*lr))/(pow(dx,2)*(pow(dy - dyaw*lr, 2)/pow(dx,2) + 1)) - dyaw*m + (Caf*cos(delta)*(dy + dyaw*lf))/(pow(dx,2)*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  double A22 = (Car/(dx*(pow(dy - dyaw*lr, 2)/pow(dx,2) + 1)) + (Caf*cos(delta))/(dx*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  double A23 = 0.0;
  double A24 = (dx*m - (Car*lr)/(dx*(pow(dy - dyaw*lr, 2)/pow(dx,2) + 1)) + (Caf*lf*cos(delta))/(dx*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/m;
  
  double A31 = 0.0; double A32 = 0.0; double A33 = 0.0; double A34 = 1.0;
  
  double A41 = ((Car*lr*(dy - dyaw*lr))/(pow(dx,2)*(pow(dy - dyaw*lr,2)/pow(dx,2) + 1)) - (Caf*lf*cos(delta)*(dy + dyaw*lf))/(pow(dx,2)*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/Izz;
  double A42 = -((Car*lr)/(dx*(pow(dy - dyaw*lr,2)/pow(dx,2) + 1)) - (Caf*lf*cos(delta))/(dx*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/Izz;
  double A43 = 0.0;
  double A44 = ((Car*pow(lr,2))/(dx*(pow(dy - dyaw*lr,2)/pow(dx,2) + 1)) + (Caf*pow(lf,2)*cos(delta))/(dx*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/Izz;

  double B11 = 1/m;
  double B12 = (Caf*sin(delta) + Caf*cos(delta)*(delta - atan((dy + dyaw*lf)/dx)))/m; // TODO: Add slip ratio dynamics to the FL (- Clf*lambda_f*sin(delta))/m);
  double B21 = 0.0;
  double B22 = (Caf*sin(delta)*(delta - atan((dy + dyaw*lf)/dx)) - Caf*cos(delta))/m; // + Clf*lambda_f*cos(delta))/m;
  double B31 = 0.0; double B32 = 0.0;
  double B41 = 0.0;
  double B42 = (lf*(Caf*sin(delta)*(delta - atan((dy + dyaw*lf)/dx)) - Caf*cos(delta)))/Izz; //+ Clf*lambda_f*cos(delta)))/Izz;

  // Generate the state-space matrices with the previously defined coefficients from linearization:

  Eigen::Matrix<double, 7, 7> A;
  A(0, 0) = A11; A(0, 1) = A12; A(0, 2) = A13; A(0, 3) = A14; A(0, 4) = 1.0; A(0, 5) = 0.0; A(0, 6) = 0.0;
  A(1, 0) = A21; A(1, 1) = A22; A(1, 2) = A23; A(1, 3) = A24; A(1, 4) = 0.0; A(1, 5) = 0.0; A(1, 6) = 0.0;
  A(2, 0) = A31; A(2, 1) = A32; A(2, 2) = A33; A(2, 3) = A34; A(2, 4) = 0.0; A(2, 5) = 0.0; A(2, 6) = 0.0;
  A(3, 0) = A41; A(3, 1) = A42; A(3, 2) = A43; A(3, 3) = A44; A(3, 4) = 0.0; A(3, 5) = 1.0; A(3, 6) = 0.0;
  A(4, 0) = 0.0; A(4, 1) = 0.0; A(4, 2) = 0.0; A(4, 3) = 0.0; A(4, 4) = 0.0; A(4, 5) = 0.0; A(4, 6) = 0.0;
  A(5, 0) = 0.0; A(5, 1) = 0.0; A(5, 2) = 0.0; A(5, 3) = 0.0; A(5, 4) = 0.0; A(5, 5) = 0.0; A(5, 6) = 1.0;
  A(6, 0) = 0.0; A(6, 1) = 0.0; A(6, 2) = 0.0; A(6, 3) = 0.0; A(6, 4) = 0.0; A(6, 5) = 0.0; A(6, 6) = 0.0; 
  
  Eigen::Matrix<double, 7, 2> B;
  B(0, 0) = B11; B(0, 1) = B12;
  B(1, 0) = B21; B(1, 1) = B22;
  B(2, 0) = B31; B(2, 1) = B32; 
  B(3, 0) = B41; B(3, 1) = B42;
  B(4, 0) = 0.0; B(4, 1) = 0.0;
  B(5, 0) = 0.0; B(5, 1) = 0.0;
  B(6, 0) = 0.0; B(6, 1) = 0.0;

  Eigen::Matrix<double, 4, 7> Cd;
  Cd(0, 0) = 1.0; Cd(0, 1) = 0.0; Cd(0, 2) = 0.0; Cd(0, 3) = 0.0; Cd(0, 4) = 0.0; Cd(0, 5) = 0.0; Cd(0, 6) = 0.0;
  Cd(1, 0) = 0.0; Cd(1, 1) = 1.0; Cd(1, 2) = 0.0; Cd(1, 3) = 0.0; Cd(1, 4) = 0.0; Cd(1, 5) = 0.0; Cd(1, 6) = 0.0;
  Cd(2, 0) = 0.0; Cd(2, 1) = 0.0; Cd(2, 2) = 1.0; Cd(2, 3) = 0.0; Cd(2, 4) = 0.0; Cd(2, 5) = 0.0; Cd(2, 6) = 0.0;
  Cd(3, 0) = 0.0; Cd(3, 1) = 0.0; Cd(3, 2) = 0.0; Cd(3, 3) = 1.0; Cd(3, 4) = 0.0; Cd(3, 5) = 0.0; Cd(3, 6) = 0.0;

  // Discretize: 
  Eigen::MatrixXd Ident7 = Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd Ident4 = Eigen::MatrixXd::Identity(4, 4);

  Eigen::Matrix<double, 7, 7> Ad;
  Ad = Ident7 + A*dt;

  Eigen::Matrix<double, 7, 2> Bd;
  Bd = B*dt;

  Eigen::Matrix<double, 7, 7> Q;
  Q = 0.2*Ident7;

  Eigen::Matrix<double, 4, 4> R;
  R = 0.01*Ident4;


  // Construct u and y vectors:
  Eigen::Matrix<double, 2, 1> u;
  u(0) = this->F_traction; 
  u(1) = this->steer;

  Eigen::Matrix<double, 4, 1> y;
  y(0) = dx;
  y(1) = dy;
  y(2) = yaw;
  y(3) = dyaw;

  // Predict:
  Eigen::Matrix<double, 7, 1> x_hat_apriori;
  x_hat_apriori = Ad*x_hat_prev_local + Bd*u;

  Eigen::Matrix<double, 7, 7> Pk_;  
  Pk_ = Ad*Pk_prev_local*Ad.transpose() + Q;

  // Update:
  Eigen::Matrix<double, 7, 4> Kk; // Kalman Gain
  Eigen::Matrix<double, 4, 4> product; 

  product = Cd*Pk_*Cd.transpose() + R;
  Kk = Pk_*Cd.transpose()*product.inverse(); 

  Eigen::Matrix<double, 7, 1> x_hat;
  x_hat = x_hat_apriori + Kk*(y - Cd*x_hat_apriori);
  
  // Return covariance matrix for next iteration:
  Eigen::Matrix<double, 7, 7> Pk_local;
  Pk_local = (Ident7 - Kk*Cd)*Pk_;


  return std::make_pair(x_hat, Pk_local); //x_hat; 

}


std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>> StanleyADRC::KalmanExtendedObserver(Eigen::Matrix<double, 6, 1> x_hat_prev_local, Eigen::Matrix<double, 6, 6> Pk_prev_local) {

  double delta = this->steer;
  double T_traction = this->F_traction / VP_.rearWheelRadius;
  double dy = this->lat_speed;
  double yaw = this->heading;
  double dyaw = this->yaw_rate;
  double eps = 0.01;
  double dx = this->speed_ + eps; // FIXME: Add epsilon?

  double A11 = (Caf*sin(delta)*(dy + dyaw*lf))/(pow(dx,2)*m*(pow((dy + dyaw*lf), 2)/pow(dx,2) + 1));
  double A12 = (dyaw*m - (Caf*sin(delta))/(dx*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  double A13 = 0.0;
  double A14 = (dy*m - (Caf*lf*sin(delta))/(dx*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  
  double A21 = -((Car*(dy - dyaw*lr))/(pow(dx,2)*(pow(dy - dyaw*lr, 2)/pow(dx,2) + 1)) - dyaw*m + (Caf*cos(delta)*(dy + dyaw*lf))/(pow(dx,2)*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  double A22 = (Car/(dx*(pow(dy - dyaw*lr, 2)/pow(dx,2) + 1)) + (Caf*cos(delta))/(dx*(pow(dy + dyaw*lf, 2)/pow(dx,2) + 1)))/m;
  double A23 = 0.0;
  double A24 = (dx*m - (Car*lr)/(dx*(pow(dy - dyaw*lr, 2)/pow(dx,2) + 1)) + (Caf*lf*cos(delta))/(dx*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/m;
  
  double A31 = 0.0; double A32 = 0.0; double A33 = 0.0; double A34 = 1.0;
  
  double A41 = ((Car*lr*(dy - dyaw*lr))/(pow(dx,2)*(pow(dy - dyaw*lr,2)/pow(dx,2) + 1)) - (Caf*lf*cos(delta)*(dy + dyaw*lf))/(pow(dx,2)*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/Izz;
  double A42 = -((Car*lr)/(dx*(pow(dy - dyaw*lr,2)/pow(dx,2) + 1)) - (Caf*lf*cos(delta))/(dx*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/Izz;
  double A43 = 0.0;
  double A44 = ((Car*pow(lr,2))/(dx*(pow(dy - dyaw*lr,2)/pow(dx,2) + 1)) + (Caf*pow(lf,2)*cos(delta))/(dx*(pow(dy + dyaw*lf,2)/pow(dx,2) + 1)))/Izz;
  
  double B11 = 1/m;
  double B12 = (Caf*sin(delta) + Caf*cos(delta)*(delta - atan((dy + dyaw*lf)/dx)))/m; // TODO: Add slip ratio dynamics to the FL (- Clf*lambda_f*sin(delta))/m);
  double B21 = 0.0;
  double B22 = (Caf*sin(delta)*(delta - atan((dy + dyaw*lf)/dx)) - Caf*cos(delta))/m; // + Clf*lambda_f*cos(delta))/m;
  double B31 = 0.0; double B32 = 0.0;
  double B41 = 0.0;
  double B42 = (lf*(Caf*sin(delta)*(delta - atan((dy + dyaw*lf)/dx)) - Caf*cos(delta)))/Izz; //+ Clf*lambda_f*cos(delta)))/Izz;

  // Generate the state-space matrices with the previously defined coefficients from linearization:

  Eigen::Matrix<double, 6, 6> A;
  A(0, 0) = A11; A(0, 1) = A12; A(0, 2) = A13; A(0, 3) = A14; A(0, 4) = 1.0; A(0, 5) = 0.0;
  A(1, 0) = A21; A(1, 1) = A22; A(1, 2) = A23; A(1, 3) = A24; A(1, 4) = 0.0; A(1, 5) = 0.0;
  A(2, 0) = A31; A(2, 1) = A32; A(2, 2) = A33; A(2, 3) = A34; A(2, 4) = 0.0; A(2, 5) = 0.0;
  A(3, 0) = A41; A(3, 1) = A42; A(3, 2) = A43; A(3, 3) = A44; A(3, 4) = 0.0; A(3, 5) = 1.0;
  A(4, 0) = 0.0; A(4, 1) = 0.0; A(4, 2) = 0.0; A(4, 3) = 0.0; A(4, 4) = 0.0; A(4, 5) = 0.0;
  A(5, 0) = 0.0; A(5, 1) = 0.0; A(5, 2) = 0.0; A(5, 3) = 0.0; A(5, 4) = 0.0; A(5, 5) = 0.0;
  
  Eigen::Matrix<double, 6, 2> B;
  B(0, 0) = B11; B(0, 1) = B12;
  B(1, 0) = B21; B(1, 1) = B22;
  B(2, 0) = B31; B(2, 1) = B32; 
  B(3, 0) = B41; B(3, 1) = B42;
  B(4, 0) = 0.0; B(4, 1) = 0.0;
  B(5, 0) = 0.0; B(5, 1) = 0.0;

  Eigen::Matrix<double, 4, 6> Cd;
  Cd(0, 0) = 1.0; Cd(0, 1) = 0.0; Cd(0, 2) = 0.0; Cd(0, 3) = 0.0; Cd(0, 4) = 0.0; Cd(0, 5) = 0.0;
  Cd(1, 0) = 0.0; Cd(1, 1) = 1.0; Cd(1, 2) = 0.0; Cd(1, 3) = 0.0; Cd(1, 4) = 0.0; Cd(1, 5) = 0.0;
  Cd(2, 0) = 0.0; Cd(2, 1) = 0.0; Cd(2, 2) = 1.0; Cd(2, 3) = 0.0; Cd(2, 4) = 0.0; Cd(2, 5) = 0.0;
  Cd(3, 0) = 0.0; Cd(3, 1) = 0.0; Cd(3, 2) = 0.0; Cd(3, 3) = 1.0; Cd(3, 4) = 0.0; Cd(3, 5) = 0.0;

  // Discretize: 
  Eigen::MatrixXd Ident6 = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd Ident4 = Eigen::MatrixXd::Identity(4, 4);

  Eigen::Matrix<double, 6, 6> Ad;
  Ad = Ident6 + A*dt;

  Eigen::Matrix<double, 6, 2> Bd;
  Bd = B*dt;

  Eigen::Matrix<double, 6, 6> Q;
  Q = 0.2*Ident6;

  Eigen::Matrix<double, 4, 4> R;
  R = 0.01*Ident4;


  // Construct u and y vectors:
  Eigen::Matrix<double, 2, 1> u;
  u(0) = this->F_traction; 
  u(1) = this->steer;

  Eigen::Matrix<double, 4, 1> y;
  y(0) = dx;
  y(1) = dy;
  y(2) = yaw;
  y(3) = dyaw;

  // Predict:
  Eigen::Matrix<double, 6, 1> x_hat_apriori;
  x_hat_apriori = Ad*x_hat_prev_local + Bd*u;

  Eigen::Matrix<double, 6, 6> Pk_;  
  Pk_ = Ad*Pk_prev_local*Ad.transpose() + Q;

  // Update:
  Eigen::Matrix<double, 6, 4> Kk; // Kalman Gain
  Eigen::Matrix<double, 4, 4> product; 

  product = Cd*Pk_*Cd.transpose() + R;
  Kk = Pk_*Cd.transpose()*product.inverse(); 

  Eigen::Matrix<double, 6, 1> x_hat;
  x_hat = x_hat_apriori + Kk*(y - Cd*x_hat_apriori);

  Eigen::Matrix<double, 6, 6> Pk_local;
  Pk_local = (Ident6 - Kk*Cd)*Pk_;


  return std::make_pair(x_hat, Pk_local);

}

double _enact_saturation(double value, double sat) {
  return std::min(std::abs(value), sat)* value/std::abs(value);
}

/** 
 * Ensures the rate of change of value does not exceed sat
 * value: the value we don't want to change faster than sat
 * prev_value: the value of value from the previous loop
 * derivative: (value-prev_value)/dt
 * sat: the maximum derivative
*/
double _enact_derivative_saturation(double value, double prev_value, double derivative, double sat) {
  return std::abs(derivative) <= std::abs(sat) ? value : prev_value + sat*derivative/std::abs(derivative);
}

void StanleyADRC::calculateFB(double dt) {
  
  /*double lf = this->get_parameter("vehicle.cg_to_front").as_double();
  double lr = this->get_parameter("vehicle.cg_to_rear").as_double();
  double Caf = this->get_parameter("vehicle.tire_stiffness_front").as_double();
  double Car = this->get_parameter("vehicle.tire_stiffness_rear").as_double();
  double m = this->get_parameter("vehicle.mass").as_double();
  double Izz = this->get_parameter("vehicle.inertia").as_double();*/

  // Compute the desired yaw rate:
  this->desired_yaw_rate = this->speed_*this->curvature_; 
  // RCLCPP_INFO(this->get_logger(), "Desired yaw rate: %f", this->desired_yaw_rate);

  // Speed error:
  this->speed_error = this->desired_velocity - this->speed_;

  // Compute lookahead error:
  double del = look_ahead_error_history[lae_history_counter];
  look_ahead_error_history[lae_history_counter] = this->lookahead_error.data;
  lae_history_counter = (lae_history_counter + 1) % 4;
  lae_history_avg = lae_history_avg - 0.25 * del + 0.25 * this->lookahead_error.data;

  // Implement the Kalman Filter with the ESO:
  Eigen::Matrix<double, 7, 1> x_hat; // Initialize vector of estimated states 
  Eigen::Matrix<double, 7, 7> Pk; 
  auto result = KalmanExtendedObserverRamp(this->x_hat_prev, this->Pk_prev); //KalmanExtendedObserver();

  x_hat = result.first;
  Pk = result.second;

  double eps = 0.01;
  double delta = this->steer;
  double dy = this->lat_speed;
  double yaw = this->heading;
  double dyaw = this->yaw_rate;
  double dx = this->speed_ + eps; 

  // Feedback linearization terms for yaw control:
  a22 = - (2*Caf + 2*Car) / (m*dx);
  a24 = -dx - (2*Caf*lf - 2*Car*lr) / (m*dx);
  a42 = -(2*lf*Caf-2*lr*Car)/(Izz*dx); 
  a44 = -(2*lf*lf*Caf+2*lr*lr*Car)/(Izz*dx);



  // PP PID Control
  // float controller_drive =  0.0 + K1*(this->desired_yaw_rate - this->yaw_rate) + K2*(this->  - this->heading);


  double K1 = this->get_parameter("K1").as_double();
  double K1d = this->get_parameter("K1d").as_double();
  double K1sat = this->get_parameter("K1sat").as_double();
  double K2 = this->get_parameter("K2").as_double();
  double K2d = this->get_parameter("K2d").as_double();
  double K3 = this->get_parameter("K3").as_double();
  double K3sat = this->get_parameter("K3sat").as_double();
  double wheelbase = this->get_parameter("vehicle.wheelbase").as_double();
  int cascade_mod = this->get_parameter("cascade_mod").as_int();

  double control_tune_speed = std::max(std::abs(this->speed_), 10.0);

  // normalize parameters by speed
  K2 *= 50/control_tune_speed;
  K2d *= 50/control_tune_speed;

  #define USE_FERDINAND

  // Ferdinand controller
  #ifdef USE_FERDINAND

  

  iteration_counter++; // NOTE: this will take around 10^9 years to overflow, so no need to check

  int lat_cascade_mod_value = cascade_mod; // TODO this maybe should be dependent on speed
  if (iteration_counter % lat_cascade_mod_value == 0) { // 100hz mod 3 = ~30hz
    double lat_err = this->lat_error.data;
    
    double lat_err_controller_drive = K1*lat_err - K1d*(lat_err-prev_lat_error)/(dt*lat_cascade_mod_value); // control based on lateral error
    
    // yaw_ref_correction = std::asin(std::clamp(lat_err_controller_drive/this->speed_,-1.0,1.0)); // convert lateral velocity to yaw angle
    yaw_ref_correction = std::atan(lat_err_controller_drive/(1+this->speed_));
    yaw_ref_correction = _enact_saturation(yaw_ref_correction, K1sat); // enact saturation
    prev_lat_error = lat_err;

  }
  
  float yaw_error = this->yaw_ref + yaw_ref_correction - this->heading;

  // float yaw_error_change = yaw_error-this->prev_yaw_error;
  // float max_yaw_error_change_allowed = 0.02F; //(this->steer)*0.1;
  // RCLCPP_INFO(this->get_logger(), "Value %f, prev_value %f, derivative %f, sat %f", yaw_error, this->prev_yaw_error, yaw_error_change, max_yaw_error_change_allowed);
  // float filtered_yaw_error = _enact_derivative_saturation(yaw_error, this->prev_yaw_error, yaw_error_change, max_yaw_error_change_allowed); // HACK this filtering kinda stinks
  
  float yaw_rate_correction = K2*(yaw_error) - K2d*(this->yaw_rate - 0*this->curvature_*this->speed_);

  // set controller drive if yaw_rate_correction is not NaN
  float controller_drive = yaw_rate_correction == yaw_rate_correction ? yaw_rate_correction : prev_controller_drive; // rad/s  // TODO add saturation based on vehicle stability limits + friction circle

  double curv_injection = K3*atan(this->curvature_*wheelbase); //FIXME maybe should be atan
  
  prev_yaw_error = yaw_error;

  // set steering
  double steering = (Izz / (2*lf*Caf))*controller_drive + curv_injection; // No ADRC
  // double steering = (Izz / (2*lf*Caf))*( controller_drive - x_hat(5)) + curv_injection; // With ADRC

  // dynamics limits saturation
  // steering = _enact_saturation(steering, 5*std::abs(K3*this->curvature_) + 0.02/2); // saturation relative to curvature
  // steering = _enact_saturation(steering, 0.1 * 80/control_tune_speed); // saturation based on estimated friction circle

  // assign to steer
  this->steer = steering;

  #else // Stanley controller

  double curv_injection = 0.0;
  double speed = this->speed_;
  double steer = this->steer;
  double yaw_error = this->yaw_ref - this->heading;
  double crosstrack_error_derivative = speed*sin(yaw_error-this->steer); // FIXME check sign of this->steer
  double yaw_rate = -speed*sin(steer) / (wheelbase);

  double lat_err_control = atan( (200 * K1 * this->lat_error.data) / ( 1 + speed) );
  // double lat_err_control = 0;
  double psi = yaw_error;
  
  steer = K2*yaw_error + lat_err_control + K2d * (yaw_rate - this->curvature_*this->speed_);

  this->steer = (Izz / (2*lf*Caf))*steer;

  #endif



  // this->steer = (Izz / (2*lf*Caf))*(-a42*dy - a44*dyaw - x_hat(5) + controller_drive);
  // this->steer = (Izz / (2*lf*Caf))*controller_drive;

  if (this->desired_velocity <= 0.5) {
    this->F_traction = m*(- dy*dyaw + K1l*this->speed_error);
  }
  else{
    this->F_traction = m*(-x_hat(4) - dy*dyaw + K1l*this->speed_error);
  }

  RCLCPP_DEBUG(this->get_logger(), "Force traction: %f", this->F_traction);
  RCLCPP_DEBUG(this->get_logger(), "Long. Disturbance: %f", x_hat(4));

  if (this->F_traction > 7800.0) {
    this->F_traction = 7800.0;
  } 

  if (this->F_traction < -7000.0) {
    this->F_traction = -7000.0;
  }

  

  RCLCPP_DEBUG(this->get_logger(), "Force: '%f'", this->F_traction);
  RCLCPP_DEBUG(this->get_logger(), "Heading disturbance: '%f'", x_hat(5));
  RCLCPP_DEBUG(this->get_logger(), "Desired yaw rate: '%f'", this->desired_yaw_rate);
  RCLCPP_DEBUG(this->get_logger(), "Yaw rate: '%f'", this->yaw_rate);
  
  float desired_accel = this->F_traction / this->m;

    
  std_msgs::msg::Float32 desired_accel_msg;
  
  if (! (desired_accel == desired_accel)) { // check for .nan
    desired_accel = 0.0;
    pubError(".nan in desired_acceleration", 5, 0.3);
  }
  
  // traction control
  if (this->traction_control) desired_accel = std::min(desired_accel, 0.0f);


  // set desired_accel in msgs
  desired_accel_msg.data = desired_accel;
  control_cmd_msg.accelerator_cmd = desired_accel;
  
  
  pubDesiredAccel_->publish(desired_accel_msg); // DEPRECATED with controller switching & control_cmd_msg
  //calculateThrottleBrakeFromTractionForce(this->F_traction); // TODO: Deprecate? Keep?


  RCLCPP_DEBUG(this->get_logger(), "Speed comparison: '%f, %f'", this->speed_, x_hat(0));
  RCLCPP_DEBUG(this->get_logger(), "Yaw comparison '%f, %f'", this->heading, x_hat(2));
  RCLCPP_DEBUG(this->get_logger(), "Yaw rate comparison '%f, %f'", this->yaw_rate, x_hat(3));


  // Saturation:
  if (this->steer > 0.3) { // FIXME: Don't hard code me
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Max");
    this->steer = 0.3;
  }
  if (this->steer < -0.3) { // FIXME: Don't hard code me
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Min");
    this->steer = -0.3;
  }


  // Store values for next callback after saturation:
  /*Eigen::Matrix<double, 4, 1> obs_in;
  obs_in << this->control_signal,
            this->lat_speed,
            this->heading,
            this->yaw_rate;        


  this->x_hat_prev = x_hat;
  this->obs_in_prev = obs_in;*/

  this->x_hat_prev = x_hat;
  this->Pk_prev = Pk;

  // Publish controller debugging msg:
  if (this->pub_debug_signals){
    blackandgold_msgs::msg::StanleyADRCDebug control_msg;
    control_msg.yaw_ref = yaw_ref;
    control_msg.yaw_rate_correction = yaw_rate_correction;
    control_msg.yaw_ref_correction = yaw_ref_correction;
    control_msg.controller_drive = controller_drive;
    control_msg.curv_injection = curv_injection;
    control_msg.yaw = this->heading;
    control_msg.yaw_rate_des = desired_yaw_rate;
    control_msg.yaw_rate = this->yaw_rate;
    control_msg.yaw_hat = x_hat(2);
    control_msg.yaw_rate_hat = x_hat(3);
    control_msg.xi_long_hat = x_hat(4);
    control_msg.xi_lat_hat = x_hat(5);
    control_msg.lat_error = this->lat_error.data;
    control_msg.yaw_error = yaw_error;
    control_msg.steering = this->steer;
    control_msg.traction_force = this->F_traction;
    control_msg.sideslip_angle = 0.0; // TODO: Not calculated yet. Could be very useful
    control_msg.long_speed= this->speed_;
    control_msg.long_speed_hat = x_hat(0);
    control_msg.long_speed_des = this->desired_velocity;
    control_msg.lat_speed= this->lat_speed;
    control_msg.lat_speed_hat = x_hat(1);
    control_msg.lookahead = this->lookahead;
    controlDebugSignals_->publish(control_msg);
  }

}

void StanleyADRC::calculateSteeringCmd() {
  this->steering_cmd.data = this->steer * 180.0 / M_PI;
  RCLCPP_DEBUG(this->get_logger(), "Steering cmd: '%f'", this->steering_cmd.data);
}

void StanleyADRC::setCmdsToZeros() {
  this->feedforward_ = 0.0;
  this->feedback_ = 0.0;
  this->lookahead_error.data = 0.0;
  this->curvature_ = 0.0;
  this->speed_ = 0.0;

  // reset moving average filter for derivative
  for (int i = 0; i < 4; i++) this->look_ahead_error_history[i] = 0.0;
  lae_history_avg = 0.;
  this->steering_cmd.data = 0.0;
}

void StanleyADRC::publishSteering() {
  RCLCPP_DEBUG(this->get_logger(), "Steering: '%f'", steering_cmd.data);
  
  rclcpp::Time nowTime = this->get_clock().get()->now();
  this->control_cmd_msg.stamp.sec = nowTime.seconds();
  this->control_cmd_msg.stamp.nanosec = nowTime.nanoseconds();

  float str_cmd = this->steering_cmd.data;
  
  // check for .nan
  if (! (str_cmd == str_cmd) ) {
    str_cmd = 0.0;
    pubError(".nan in steering output",5, 0.3);
  }
  this->control_cmd_msg.steering_cmd = str_cmd;
  pubControlCmd_->publish(control_cmd_msg);

}

void StanleyADRC::publishDebugSignals() {
  // check for too large lookahead error and publish to safety_node
  if (fabs(this->lookahead_error.data) > this->max_lookahead_error) {
    pubError("lookahead error too high:" + std::to_string(this->lookahead_error.data),3,1);
  }

  // check for too large lateral error
  if (fabs(this->lat_error.data) > this->max_lat_error) {
    pubError("<CRITICAL> lateral error too high: " + std::to_string(this->lat_error.data),4,10);
  }

  if (this->lat_err_history_counter > this->lat_err_history.size()-1) { 
    int laterrhistSize = this->lat_err_history.size();

    double max_one_step_lat_err_rate = this->get_parameter("max_one_step_lat_err_rate").as_double();
    double max_avg_lat_err_rate = this->get_parameter("max_avg_lat_err_rate").as_double();

    rclcpp::Duration dt = this->lat_err_time_history[laterrhistSize-1] - this->lat_err_time_history[laterrhistSize-2];
    double one_step_lat_err_rate = (this->lat_err_history[laterrhistSize-1] - this->lat_err_history[laterrhistSize-2])/(dt.seconds() + dt.nanoseconds()*1e-9 + __DBL_EPSILON__);
    dt = this->lat_err_time_history[laterrhistSize-1] - this->lat_err_time_history[0];
    double avg_lat_err_rate = (*std::max_element(this->lat_err_history.begin(),this->lat_err_history.end()) - *std::min_element(this->lat_err_history.begin(),this->lat_err_history.end()))/(dt.seconds() + dt.nanoseconds()*1e-9 + __DBL_EPSILON__); 

    if (fabs(one_step_lat_err_rate) > max_one_step_lat_err_rate || fabs(avg_lat_err_rate) > max_avg_lat_err_rate) {
      std::stringstream ss;
      ss << "<CRITICAL> rapid change in lateral error!\\nlateral error history: ";
      ss << "[";
      ss << std::setprecision(3) << this->lat_err_history[0];
      for (int i =1; i < this->lat_err_history.size()-1; i++) {
        ss <<  " ";
        ss << std::setprecision(3) << this->lat_err_history[i];
      }
      ss << "]";
      ss << "\\nlateral error time steps: ";
      ss << "[ ";
      dt = this->lat_err_time_history[1] - this->lat_err_time_history[0];
      ss << std::setprecision(3) << dt.seconds() + dt.nanoseconds()*1e-9;
      for (int i =1; i < this->lat_err_time_history.size()-1; i++) {
        dt = this->lat_err_time_history[i+1] - this->lat_err_time_history[i];
        ss << " ";
        ss << std::setprecision(3) << dt.seconds() + dt.nanoseconds()*1e-9;
      }
      ss << "]";
      this->error_msg.description = ss.str();
      this->error_msg.origin = "ferdinand_controller";
      this->error_msg.lifetime = 10;
      this->error_msg.module = "control";
      this->error_msg.severity = 4;
      this->pubErrorReporter_->publish(this->error_msg);
    }
  }

  pubLookaheadError_->publish(this->lookahead_error);
  pubLatError_->publish(this->lat_error);
}

void StanleyADRC::receiveGlobalOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
  this->odom = *msg;
  Quaternion quat;
  quat.x = this->odom.pose.pose.orientation.x;
  quat.y = this->odom.pose.pose.orientation.y;
  quat.z = this->odom.pose.pose.orientation.z;
  quat.w = this->odom.pose.pose.orientation.w;
  this->heading = quaternionToHeading(quat);
  // this->yaw_rate = odom.twist.twist.angular.z;
  this->lat_speed = odom.twist.twist.linear.y;
  RCLCPP_DEBUG(this->get_logger(), "Heading: '%f'", this->heading*180/3.14159);
};

void StanleyADRC::receiveLocalOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
  this->odom = *msg;
  this->yaw_rate = odom.twist.twist.angular.z;
};

double StanleyADRC::quaternionToHeading(const Quaternion& q) {
    EulerAngle euler;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0)
        euler.pitch = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    else
        euler.pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);

    return euler.yaw; // Other angles can be returned from this function, but we only need the heading
}

float StanleyADRC::_dist(float x, float y, float z) {
    return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
}


void StanleyADRC::calculateCurvatureVelocity(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

  float lookahead_distance =  std::pow(this->speed_,2) / 2 / this->get_parameter("max_possible_deceleration").as_double();

  float max_curv = 0.0F;
  float running_distance = 0.001F; // distance down path

  float curr_curv, curr_max_speed, min_decel = 0.0, deltaV;
  for (int i = 0; running_distance < lookahead_distance && i < ( (int) (msg->data).size() ) && i < ( (int) (path_msg->poses).size()); i++) {
    curr_curv = (msg->data)[i];

    curr_max_speed = std::sqrt(this->get_parameter("curv_to_velocity_constant").as_double() / (abs(curr_curv)+0.000000001));

    max_curv = std::max(abs(curr_curv), abs(max_curv));

    // calculate how far down path we are & add it to the running_distance
    running_distance += _dist((path_msg->poses)[i+1].pose.position.x - (path_msg->poses)[i].pose.position.x,(path_msg->poses)[i+1].pose.position.y - (path_msg->poses)[i].pose.position.y,(path_msg->poses)[i+1].pose.position.z - (path_msg->poses)[i].pose.position.z);
    
    // calculate the minimum deceleration to reach the max_speed at this point down the path
    deltaV = (curr_max_speed - this->speed_); 
    min_decel = std::max(min_decel, deltaV*deltaV / 2 / std::max(running_distance,10.0F));
  }

  RCLCPP_DEBUG(this->get_logger(), "Max curv: '%f,%f,%f'", max_curv, min_decel, lookahead_distance);
 
  try {
    this->max_curv_velocity_message.desired_speed =  this->get_parameter("curv_to_velocity_constant").as_double() / (abs(max_curv)+0.000000001); // add a tiny amount to avoid divide by zero
    this->max_curv_velocity_message.desired_acceleration = min_decel;
    this->pubCurvDesiredVelocity_->publish(this->max_curv_velocity_message);
  }catch(int a) {
    pubError("exception in calculating max_curv_velocity (you probably set the curv_to_velocity_constant parameter to an invalid value)",5,10.0F);
  }

  this->path_curvatures = *msg;

  
  /*
  RCLCPP_INFO(this->get_logger(), "Curvature: '%f'", this->curvature_);
  RCLCPP_INFO(this->get_logger(), "Index: '%i'", idx);
  */
}

void StanleyADRC::receivePath(const nav_msgs::msg::Path::SharedPtr msg) {
  // Determines lookahead distance based on speed and bounds

  this->path_msg = msg;
  double lookahead_distance = 0.1; // std::max(min_la, std::min(max_la, this->speed_ * la_ratio));

  // Unpacks the message and finds the index correlated to the lookahead distance
  std::vector<geometry_msgs::msg::PoseStamped> path = msg->poses;
  // Sets the lookahead and lateral error
  if (path.size() < 2) {
    // an empty path message is NOT a valid path. Do NOT update clock in this case.
    this->lookahead_error.data = 0.0;
    pubError("Received path is empty!", 5, 0.5);
    return;
  }
  //int idx;

  // // TODO: improve lateral error calculation
  // if (path.size() == 1) {
  //   this->lat_error.data = path[0].pose.position.y;
  // } else {
  //   double diff_x = path[1].pose.position.x - path[0].pose.position.x;
  //   double diff_y = path[1].pose.position.y - path[0].pose.position.y;
  //   double norm_factor = 1. / std::sqrt(diff_x * diff_x + diff_y * diff_y);
  //   double norm_x = diff_x * norm_factor;
  //   double norm_y = diff_y * norm_factor;
  //   this->lat_error.data = -path[0].pose.position.x * norm_y + path[0].pose.position.y * norm_x;
  // }

  double K4 = this->get_parameter("K4").as_double();
  double K4_curv = this->get_parameter("K4_curv").as_double();


  double wheelbase = this->get_parameter("vehicle.wheelbase").as_double();

  // how many points ahead we will use to calculate our desired heading
  
  double heading_lookahead_meters = std::max(0.0,K4*this->speed_*dt);
  double curvature_lookahead_meters = std::max(0.0,K4_curv*this->speed_*dt) + wheelbase/2.0;

  
  // the length in points of the vector we will use to compute desired heading. More points will be smoother but less responsive
  unsigned const int heading_width = 2;
  unsigned const int curvature_width = 8;

  int heading_lookahead = path.size()-heading_width-1;
  int curvature_lookahead = path.size()-2*curvature_width-1;

  if (heading_lookahead < 0 || curvature_lookahead < 0) {
    pubError("Path has too few points!", 5, 5);
  }

  double lateral_error = 0.0;

  bool lat_error_set = false;
  bool heading_lookahead_set = false;
  bool curvature_lookahead_set = false;
  float running_distance = 0.0;
  for (int i = 0; i < path.size()-1; i++) {

    if (! lat_error_set && running_distance > wheelbase/2) {
      lat_error_set = true;
      lateral_error = path[i].pose.position.y;
      this->lat_error.data = lateral_error;
    }
    if (! heading_lookahead_set && running_distance > heading_lookahead_meters) {
      heading_lookahead_set = true;
      heading_lookahead = i;
    }
    if (! curvature_lookahead_set && running_distance > curvature_lookahead_meters) {
      curvature_lookahead_set = true;
      curvature_lookahead = i;
    }

    if (lat_error_set && heading_lookahead_set && curvature_lookahead_set) break;

    
    running_distance += std::sqrt(std::pow(path[i].pose.position.x - path[i+1].pose.position.x,2) + std::pow(path[i].pose.position.y - path[i+1].pose.position.y,2) ); 

  }
  this->lookahead = heading_lookahead_meters;

  float meters_per_point = std::sqrt(std::pow(path[0].pose.position.x - path[1].pose.position.x,2) + std::pow(path[0].pose.position.y - path[1].pose.position.y,2));

  // compute reference heading
  double path_heading = std::atan2(path[heading_width+heading_lookahead].pose.position.y - path[heading_lookahead].pose.position.y, path[heading_width+heading_lookahead].pose.position.x - path[heading_lookahead].pose.position.x);
  this->yaw_ref = path_heading + this->heading;
  
  // calculate path curvature
  const bool USE_IN_HOUSE_CURVATURE_CALCULATION = true;

  if (this->path_curvatures.data.size() > curvature_lookahead && ! USE_IN_HOUSE_CURVATURE_CALCULATION) {
    if (! USE_IN_HOUSE_CURVATURE_CALCULATION) pubError("not receiving path curvature, or path curvature lookahead too small", 1, 1);
    
    this->curvature_ = (this->path_curvatures.data)[curvature_lookahead];
  }else{
  
    // compute path curvature (rad / longitudinal_meter)
    double curv_heading_1 = std::atan2(path[curvature_width+curvature_lookahead].pose.position.y - path[curvature_lookahead].pose.position.y, path[curvature_width+curvature_lookahead].pose.position.x - path[curvature_lookahead].pose.position.x);
    double curv_heading_2 = std::atan2(path[2*curvature_width+curvature_lookahead].pose.position.y - path[curvature_width+curvature_lookahead].pose.position.y, path[2*curvature_width+curvature_lookahead].pose.position.x - path[curvature_width+curvature_lookahead].pose.position.x);

    this->curvature_ = 2 * (curv_heading_2 - curv_heading_1) / ( meters_per_point*curvature_width );
  }

  std_msgs::msg::Float32 curvature_msg;
  curvature_msg.data = this->curvature_;
  pubCurvature->publish(curvature_msg);


  // publish lookahead markers
  visualization_msgs::msg::Marker curvature_lookahead_marker;
  rclcpp::Time nowTime = this->get_clock().get()->now();
  curvature_lookahead_marker.header.stamp.sec = nowTime.seconds();
  curvature_lookahead_marker.header.stamp.nanosec = nowTime.nanoseconds();
  curvature_lookahead_marker.header.frame_id = "vehicle";
  curvature_lookahead_marker.type = 1;
  curvature_lookahead_marker.pose.position = path[curvature_lookahead].pose.position;
  curvature_lookahead_marker.scale.x = 0.1;
  curvature_lookahead_marker.scale.y = 0.1;
  curvature_lookahead_marker.scale.z = 0.1;
  curvature_lookahead_marker.lifetime.nanosec = 1000;
  curvature_lookahead_marker.color.r = 100;
  curvature_lookahead_marker.color.a = 10;
  pubCurvatureLookaheadMarker_->publish(curvature_lookahead_marker);

  visualization_msgs::msg::Marker heading_lookahead_marker;

  heading_lookahead_marker.header.stamp.sec = nowTime.seconds();
  heading_lookahead_marker.header.stamp.nanosec = nowTime.nanoseconds();
  heading_lookahead_marker.header.frame_id = "vehicle";
  heading_lookahead_marker.type = 1;
  heading_lookahead_marker.pose.position = path[heading_lookahead].pose.position;
  heading_lookahead_marker.scale.x = 0.1;
  heading_lookahead_marker.scale.y = 0.1;
  heading_lookahead_marker.scale.z = 0.1;
  heading_lookahead_marker.lifetime.nanosec = 1000;
  heading_lookahead_marker.color.b = 100;
  heading_lookahead_marker.color.a = 10;
  pubHeadingLookaheadMarker_->publish(heading_lookahead_marker);




  this->recv_time_ = rclcpp::Clock().now();
}

std::tuple<int, double> StanleyADRC::findLookaheadIndex(
    std::vector<geometry_msgs::msg::PoseStamped > refPath, double desLookaheadValue) {
  // calculate frenet distance iteratively.
  double cumulativeDist;
  if (refPath.size() == 1) {
    cumulativeDist = 0.;
  } else {
    double diff_x = refPath[1].pose.position.x - refPath[0].pose.position.x;
    double diff_y = refPath[1].pose.position.y - refPath[0].pose.position.y;
    double norm_factor = 1. / std::sqrt(diff_x * diff_x + diff_y * diff_y);
    double norm_x = diff_x * norm_factor;
    double norm_y = diff_y * norm_factor;
    cumulativeDist = refPath[0].pose.position.x * norm_x + refPath[0].pose.position.y * norm_y;
  }
  double lastCumulativeDist = 0.;
  size_t i;
  for (i = 0; i < (refPath.size() - 1) && cumulativeDist < desLookaheadValue; i++) {
    double diffX = refPath[i + 1].pose.position.x - refPath[i].pose.position.x;
    double diffY = refPath[i + 1].pose.position.y - refPath[i].pose.position.y;
    lastCumulativeDist = cumulativeDist;
    cumulativeDist += std::sqrt(diffX * diffX + diffY * diffY);
  }
  return {i, (cumulativeDist - desLookaheadValue) /
                 (cumulativeDist - lastCumulativeDist + __DBL_EPSILON__)};
}

void StanleyADRC::receiveWheelVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) { // NOTE: But then, when the wheels slide, this is not valid
  const double kphToMps = 1.0 / 3.6;
  this->rear_left_wheel_speed = msg->rear_left;
  this->rear_right_wheel_speed = msg->rear_right;
  this->speed_ = (this->rear_left_wheel_speed + this->rear_right_wheel_speed) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
}


void StanleyADRC::receiveVelocity(const std_msgs::msg::Float32::SharedPtr msg) {
  // this->speed_ = msg->data;
  if (2*msg->data+1 < this->speed_) { // we have lost traction in rear wheels
    pubError("we have lost traction!", blackandgold_msgs::msg::ErrorReport::ERROR_FAULT, 3);
    this->traction_control = true;
  }else{
    this->traction_control = false;
  }
}


void StanleyADRC::pubError(std::string error_reason, int8_t error_severity, float error_lifetime) {
  rclcpp::Time nowTime = this->get_clock().get()->now();
  this->error_msg.header.stamp.sec = nowTime.seconds();
  this->error_msg.header.stamp.nanosec = nowTime.nanoseconds();

  this->error_msg.description = error_reason;
  this->error_msg.origin = "ferdinand_controller";
  this->error_msg.lifetime = error_lifetime;
  this->error_msg.module = "control";
  this->error_msg.severity = error_severity;
  this->pubErrorReporter_->publish(this->error_msg);
}

}  // end namespace control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::StanleyADRC>());
  rclcpp::shutdown();
  return 0;
}
