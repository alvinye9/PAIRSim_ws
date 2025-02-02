/* Copyright 2021 Will Bryan
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

#include <Coupled_ADRC.hpp>

namespace control {

void CoupledADRC::controlCallback() {
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

void CoupledADRC::calculateFFW() {
  // Desired yaw rate from feedforward
  double lf = this->get_parameter("vehicle.cg_to_front").as_double();
  double lr = this->get_parameter("vehicle.cg_to_rear").as_double();
  double Caf = this->get_parameter("vehicle.tire_stiffness_front").as_double();
  double Car = this->get_parameter("vehicle.tire_stiffness_rear").as_double();
  double m = this->get_parameter("vehicle.mass").as_double();
  double Izz = this->get_parameter("vehicle.inertia").as_double();
  //double K3 = this->get_parameter("K3").as_double();

  double mf = m*lf / (lf + lr);
  double mr = m*lr / (lf + lr);

  this->feedforward_ = 0.0;
  RCLCPP_DEBUG(this->get_logger(), "Ffw: '%f'", this->feedforward_);
}

void CoupledADRC::receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
  this->current_gear_ = msg->current_gear;
  this->engine_speed_ = msg->engine_rpm;
  this->engine_running_ = msg->engine_on_status;
  RCLCPP_DEBUG(this->get_logger(), "Engine RPM: '%f'", this->engine_speed_);

}

void CoupledADRC::receiveDesiredVelocity(const std_msgs::msg::Float32::SharedPtr msg) {
  this->desired_velocity = msg->data;
}

double CoupledADRC::interpolateEngineMaxTorque(double rpm){

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
    double interpolation_result = yL + slope*(rpm-xL);;
    return yL + slope*(rpm-xL);
  }
}

void CoupledADRC::calculateThrottleBrakeFromTractionForce(double Ft){
  
  double required_traction_torque = Ft * VP_.rearWheelRadius;

  std::stringstream ss;

  // TODO: Calculate required gear:
  double torqueDemandedFromEngine[this->VP_.nGear];
  //double rpmAtThisGear[this->VP_.nGear];
  double torqueMaxAtThisGear[this->VP_.nGear];
  double torqueMinAtThisGear[this->VP_.nGear];

  // Get engine min-max torques:
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

std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>> CoupledADRC::KalmanExtendedObserverRamp(Eigen::Matrix<double, 7, 1> x_hat_prev_local, 
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

  double cov_Q = this->get_parameter("cov_Q").as_double();
  double cov_R = this->get_parameter("cov_R").as_double();

  Eigen::Matrix<double, 7, 7> Q;
  Q = cov_Q*Ident7;

  Eigen::Matrix<double, 4, 4> R;
  R = cov_R*Ident4;


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


std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>> CoupledADRC::KalmanExtendedObserver(Eigen::Matrix<double, 6, 1> x_hat_prev_local, Eigen::Matrix<double, 6, 6> Pk_prev_local) {

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

void CoupledADRC::calculateFB(double dt) {

  double K1 = this->get_parameter("K1").as_double();
  double K2 = this->get_parameter("K2").as_double();
  double K3 = this->get_parameter("K3").as_double();
  double K4 = this->get_parameter("K4").as_double();
  double K1l = this->get_parameter("K1l").as_double();
  
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


  
  //this->steer =  (Izz / (2*lf*Caf))*(-a42*dy - a44*dyaw - x_hat(5) + 0.0 + K1*(this->desired_yaw_rate - this->yaw_rate)
  //  + K2*(this->yaw_ref - this->heading));

  // this->steer =  (Izz / (2*lf*Caf))*(-a42*dy - a44*dyaw - x_hat(5) + 0.0 + K1*(this->desired_yaw_rate - this->yaw_rate)
  //  + K2*(this->yaw_ref - this->heading) + K3*atan(K4*this->lookahead_error.data / dx));

  // Ordinary ADRC:
  this->steer =  (Izz / (2*lf*Caf))*(-a42*dy - a44*dyaw - x_hat(5) + 0.0 + K1*(this->desired_yaw_rate - this->yaw_rate)
   + K2*(this->yaw_ref - this->heading)); // + K3*atan(K4*this->lookahead_error.data / dx));

  // Stanley controller:
  // this->steer = (this->yaw_ref - this->heading) + atan(K4*this->lat_error.data / dx); 
    
  //   Izz / (2*lf*Caf))*(-a42*dy - a44*dyaw - x_hat(5) + 0.0 
  //  + ((2*lf*Caf)/(Izz))*(this->yaw_ref - this->heading) + atan(K4*this->lat_error.data / dx)); // + K3*atan(K4*this->lookahead_error.data / dx));

  // this->steer = K2*(this->yaw_ref - this->heading) + K3*atan(K4*this->lookahead_error.data / dx);

  //this->steer = (Izz / (2*lf*Caf))*(-a42*dy - a44*dyaw - x_hat(5)  
  //  + K1*(this->yaw_ref - this->heading) + K2* + K1*atan(this->lookahead_error.data / dx));

  // this->F_traction = - Caf*sin(delta)*(delta - atan((dy + dyaw*this->lf) / dx)) 
    // - dy*dyaw*this->m + m*(-x_hat(4) + K1l*this->speed_error);

  if (this->desired_velocity <= 0.5) {
    this->F_traction = m*(- dy*dyaw + K1l*this->speed_error);
  }
  else{ // TODO: Evaluate this implementation as it is done to avoid accumulating a huge number in the longitudinal disturbance estimation while the car is not moving.
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
    blackandgold_msgs::msg::ADRC control_msg;
    control_msg.yaw_ref = yaw_ref;
    control_msg.yaw = this->heading;
    control_msg.yaw_rate_des = this->desired_yaw_rate;
    control_msg.yaw_rate = this->yaw_rate;
    control_msg.yaw_hat = x_hat(2);
    control_msg.yaw_rate_hat = x_hat(3);
    control_msg.xi_long_hat = x_hat(4);
    control_msg.xi_lat_hat = x_hat(5);
    control_msg.lat_error = this->lat_error.data;
    control_msg.lookahead_error = this->lookahead_error.data;
    control_msg.steering = this->steer;
    control_msg.traction_force = this->F_traction;
    control_msg.sideslip_angle = 0.0; // TODO: Not calculated yet. Could be very useful
    control_msg.long_speed= this->speed_;
    control_msg.long_speed_hat = x_hat(0);
    control_msg.lat_speed= this->lat_speed;
    control_msg.lat_speed_hat = x_hat(1);
    controlDebugSignals_->publish(control_msg);
  }

  // Publish lookahead distance:
  std_msgs::msg::Float32 lookahead_distance_msg;
  lookahead_distance_msg.data = this->lookahead_distance;
  pubLookaheadDistance_->publish(lookahead_distance_msg);

}

void CoupledADRC::calculateSteeringCmd() {
  this->steering_cmd.data = this->steer * 180.0 / M_PI;
  RCLCPP_DEBUG(this->get_logger(), "Steering cmd: '%f'", this->steering_cmd.data);
}

void CoupledADRC::setCmdsToZeros() {
  this->feedforward_ = 0.0;
  this->feedback_ = 0.0;
  this->lookahead_error.data = 0.0;
  this->curvature_ = 0.0;
  this->speed_ = 0.0;

  // reset moving average filter for derivative
  for (int i = 0; i < 4; i++) this->look_ahead_error_history[i] = 0.0;
  lae_history_avg = 0.;
  void receiveOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  this->steering_cmd.data = 0.0;
}

void CoupledADRC::publishSteering() {
  RCLCPP_DEBUG(this->get_logger(), "Steering: '%f'", steering_cmd.data);

  this->control_cmd_msg.steering_cmd = this->steering_cmd.data;
  if (!this->get_parameter("mute").as_bool()) {
    pubControlCmd_->publish(control_cmd_msg);
    pubSteeringCmd_->publish(this->steering_cmd); // DEPRECATED with controller switching & control_cmd_msg
  }
}

void CoupledADRC::publishDebugSignals() {
  // check for too large lookahead error and publish to safety_node
  if (fabs(this->lookahead_error.data) > this->max_lookahead_error) {
    this->error_msg.description =
        "lookahead error too high:" + std::to_string(this->lookahead_error.data);
    this->error_msg.origin = "adrc_lat_control_node"; // FIXME fix this name
    this->error_msg.lifetime = 1;
    this->error_msg.module = "control";
    this->error_msg.severity = 3;
    this->pubErrorReporter_->publish(this->error_msg);
  }

  // check for too large lateral error
  if (fabs(this->lat_error.data) > this->max_lat_error) {
    this->error_msg.description =
        "<CRITICAL> lateral error too high: " + std::to_string(this->lat_error.data);
    this->error_msg.origin = "adrc_lat_control_node"; // FIXME fix this name
    this->error_msg.lifetime = 10;
    this->error_msg.module = "control";
    this->error_msg.severity = 4;
    this->pubErrorReporter_->publish(this->error_msg);
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
      this->error_msg.origin = "adrc_lat_control_node";
      this->error_msg.lifetime = 10;
      this->error_msg.module = "Control";
      this->error_msg.severity = 4;
      this->pubErrorReporter_->publish(this->error_msg);
    }
  }

  pubLookaheadError_->publish(this->lookahead_error);
  pubLatError_->publish(this->lat_error);
}

void CoupledADRC::receiveOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
  this->odom = *msg;
  Quaternion quat;
  quat.x = this->odom.pose.pose.orientation.x;
  quat.y = this->odom.pose.pose.orientation.y;
  quat.z = this->odom.pose.pose.orientation.z;
  quat.w = this->odom.pose.pose.orientation.w;
  this->heading = quaternionToHeading(quat);
  this->yaw_rate = odom.twist.twist.angular.z;
  this->lat_speed = odom.twist.twist.linear.y;
  RCLCPP_DEBUG(this->get_logger(), "Heading: '%f'", this->heading*180/3.14159);
};

double CoupledADRC::quaternionToHeading(const Quaternion& q) {
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

float CoupledADRC::_dist(float x, float y, float z) {
    return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
}

/*void CoupledADRC::receiveCurvature(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  this->path_curvatures = *msg;
  this->curvature_ = path_curvatures.data[idx]; 
  std_msgs::msg::Float32 curvature_msg;
  curvature_msg.data = this->curvature_;
  pubCurvature->publish(curvature_msg);
  RCLCPP_DEBUG(this->get_logger(), "Curvature: '%f'", this->curvature_);
}*/

void CoupledADRC::calculateCurvatureVelocity(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

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
    if (! this->get_parameter("mute").as_bool()) this->pubCurvDesiredVelocity_->publish(this->max_curv_velocity_message);
  }catch(int a) {
    this->error_msg.description = "exception in calculating max_curv_velocity (you probably set the curv_to_velocity_constant parameter to an invalid value)";
    this->error_msg.origin = "adrc_control_node"; // FIXME check name here
    this->error_msg.lifetime = 10;
    this->error_msg.module = "control";
    this->error_msg.severity = 5;
    this->pubErrorReporter_->publish(this->error_msg);
  }

  this->path_curvatures = *msg;
  this->curvature_ = path_curvatures.data[idx]; 
  std_msgs::msg::Float32 curvature_msg;
  curvature_msg.data = this->curvature_;
  pubCurvature->publish(curvature_msg);
  /*
  RCLCPP_INFO(this->get_logger(), "Curvature: '%f'", this->curvature_);
  RCLCPP_INFO(this->get_logger(), "Index: '%i'", idx);
  */
}

void CoupledADRC::receivePath(const nav_msgs::msg::Path::SharedPtr msg) {
  // Determines lookahead distance based on speed and bounds

  this->path_msg = msg;
  this->lookahead_distance = std::max(min_la, std::min(max_la, this->speed_ * la_ratio));


  // Unpacks the message and finds the index correlated to the lookahead distance
  std::vector<geometry_msgs::msg::PoseStamped> path = msg->poses;
  // Sets the lookahead and lateral error
  if (!path.size()) {
    // an empty path message is NOT a valid path. Do NOT update clock in this case.
    this->lookahead_error.data = 0.0;
    return;
  }
  //int idx;
  idx = 0;
  double fraction;
  std::tie(idx, fraction) = findLookaheadIndex(path, lookahead_distance);
  if (idx >= static_cast<int>(path.size()) - 1) {
    this->lookahead_error.data = path[idx].pose.position.y;
  } else {
    double idx1_y = path[idx + 1].pose.position.y;
    double idx_y = path[idx].pose.position.y;
    this->lookahead_error.data = idx_y * fraction + idx1_y * (1. - fraction);
  }
  if (path.size() == 1) {
    this->lat_error.data = path[0].pose.position.y;
  } else {
    double diff_x = path[1].pose.position.x - path[0].pose.position.x;
    double diff_y = path[1].pose.position.y - path[0].pose.position.y;
    double norm_factor = 1. / std::sqrt(diff_x * diff_x + diff_y * diff_y);
    double norm_x = diff_x * norm_factor;
    double norm_y = diff_y * norm_factor;
    this->lat_error.data = path[0].pose.position.y; //-path[0].pose.position.x * norm_y + path[0].pose.position.y * norm_x;
  }

  // double alpha = std::atan2(path[0].pose.position.y, path[0].pose.position.x);
  double alpha = std::atan2(path[idx].pose.position.y, path[idx].pose.position.x); // idx
  this->yaw_ref = alpha + this->heading;

  // Publish lookahead marker for visualization:
  visualization_msgs::msg::Marker lookahead_marker;
  lookahead_marker.header = msg->header;
  lookahead_marker.header.stamp = rclcpp::Clock().now();
  lookahead_marker.pose.position = path[idx].pose.position;
  lookahead_marker.type = visualization_msgs::msg::Marker::CUBE;
  lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
  lookahead_marker.lifetime.sec = 0;
  lookahead_marker.lifetime.nanosec = 1000000000;
  lookahead_marker.scale.x = 5.0;
  lookahead_marker.scale.y = 5.0;
  lookahead_marker.scale.z = 5.0;
  lookahead_marker.color.r = 0.0;
  lookahead_marker.color.g = 0.0;
  lookahead_marker.color.b = 1.0;
  lookahead_marker.color.a = 0.75;
  pubLookaheadMarker_->publish(lookahead_marker);


  //   // deal with markers
  // visualization_msgs::msg::Marker m;
  // m.header = ros_pc2_in->header;
  // m.ns = "bbox";
  // m.id = i;
  // m.type = visualization_msgs::msg::Marker::CUBE;
  // m.action = visualization_msgs::msg::Marker::ADD;
  // m.pose.position.x = box.centroid.x;
  // m.pose.position.y = box.centroid.y;
  // m.pose.position.z = box.centroid.z;
  // m.pose.orientation.x = box.orientation.x;
  // m.pose.orientation.y = box.orientation.y;
  // m.pose.orientation.z = box.orientation.z;
  // m.pose.orientation.w = box.orientation.w;



  //this->yaw_ref = std::atan2(2 * lookahead_distance * sin(alpha), sqrt(pow(path[idx].pose.position.y,2) + 
  //this->yaw_ref = std::atan(2*(lf+lr)*sin(alpha) / _dist(path[idx].pose.position.x, path[idx].pose.position.y, 0.0)) + this->heading;

  //this->yaw_ref = std::atan(2*(lookahead_distance)*sin(alpha) / _dist(path[idx].pose.position.x, path[idx].pose.position.y, 0.0)) + this->heading;
  //this->yaw_ref = std::atan((lf+lr)*sin(alpha) / lookahead_distance) + this->heading;
  //this->yaw_ref = std::asin(this->speed_ * la_ratio * this->curvature_ * 0.5) + this->heading; // = asin(ld/2R) + heading
  //this->yaw_ref = std::atan2(path[idx].pose.position.y, path[idx].pose.position.x) + std::atan(lookahead_distance*this->curvature_) + this->heading;

  std_msgs::msg::Float32 yaw_ref_msg;
  yaw_ref_msg.data = this->yaw_ref;
  pubYawRef->publish(yaw_ref_msg);
  RCLCPP_DEBUG(this->get_logger(), "Yaw error: '%f'", this->yaw_error_);

  int laterrhistSize = this->get_parameter("lat_err_history_length").as_int();

  if (laterrhistSize != this->lat_err_history.size()) {
    if (laterrhistSize < this->lat_err_history.size())
      this->lat_err_history_counter = laterrhistSize;
    this->lat_err_history.resize(laterrhistSize,0.0);
    this->lat_err_time_history.resize(laterrhistSize,rclcpp::Clock().now());
  }

  if (this->lat_err_history_counter > this->lat_err_history.size()-1) {
    for (int index = 0; index < lat_err_history.size()-1; ++index){
      this->lat_err_history[index] = this->lat_err_history[index+1];
      this->lat_err_time_history[index] = this->lat_err_time_history[index+1];
    }
    this->lat_err_history[laterrhistSize-1] = this->lat_error.data;
    this->lat_err_time_history[laterrhistSize-1] = rclcpp::Clock().now();
  }
  else{
    this->lat_err_history[this->lat_err_history_counter] = this->lat_error.data;
    this->lat_err_time_history[this->lat_err_history_counter] = rclcpp::Clock().now();
    this->lat_err_history_counter++;
  }

  this->recv_time_ = rclcpp::Clock().now();
}

std::tuple<int, double> CoupledADRC::findLookaheadIndex(
    std::vector<geometry_msgs::msg::PoseStamped> refPath, double desLookaheadValue) {
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

void CoupledADRC::receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) { // FIXME: But then, when the wheels slide, this is not valid
  const double kphToMps = 1.0 / 3.6;
  this->rear_left_wheel_speed = msg->rear_left;
  this->rear_right_wheel_speed = msg->rear_right;
  this->speed_ =
    (this->rear_left_wheel_speed + this->rear_right_wheel_speed) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
}

}  // end namespace control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::CoupledADRC>());
  rclcpp::shutdown();
  return 0;
}
