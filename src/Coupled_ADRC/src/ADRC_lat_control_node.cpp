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

#include <ADRC_lat_control_node.hpp>

namespace control {

void ADRCLatControl::controlCallback() {
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

void ADRCLatControl::calculateFFW() {
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


/*void ADRCLatControl::calculateFB(double dt) {

  // Compute the desired yaw rate:
  this->desired_yaw_rate = this->speed_*this->curvature_; // dx1d

}*/

void ADRCLatControl::calculateFB(double dt) {

  /*double K1 = this->get_parameter("K1").as_double();
  double K2 = this->get_parameter("K2").as_double();
  double lf = this->get_parameter("vehicle.cg_to_front").as_double();
  double lr = this->get_parameter("vehicle.cg_to_rear").as_double();
  double Caf = this->get_parameter("vehicle.tire_stiffness_front").as_double();
  double Car = this->get_parameter("vehicle.tire_stiffness_rear").as_double();
  double m = this->get_parameter("vehicle.mass").as_double();
  double Izz = this->get_parameter("vehicle.inertia").as_double();*/


  // Compute the desired yaw rate:
  this->desired_yaw_rate = this->speed_*this->curvature_; // dx1d

  // Compute lookahead error:
  double del = look_ahead_error_history[lae_history_counter];
  look_ahead_error_history[lae_history_counter] = this->lookahead_error.data;
  lae_history_counter = (lae_history_counter + 1) % 4;
  lae_history_avg = lae_history_avg - 0.25 * del + 0.25 * this->lookahead_error.data;

  // Extended-state observer: 
  Eigen::Matrix<double, 4, 4> Aobs; // Aobs = A-L*C Discrete
  Aobs << 0.6671, 0.0, -9.1118e-15, -5.6245e-17,
          0.0, 0.6703, 0.0, 0.0,
          -5.9339e-12, 0.0, 0.4296, 0.0069,
          -2.2233e-10, 0.0, -9.6633, 0.9450
          ;

  Eigen::Matrix<double, 4,4> Bobs;
  Bobs <<  2.07, -0.5051, 0.0, -0.2383,
          0.0, 0.0, 0.3297, 0.008242,
          2.369, -0.1307, 0.0, -0.6746,
          -18.95, 1.045, 0.0, 19.62
          ;

  Eigen::Matrix<double, 4, 1> x_hat;

  x_hat = Aobs*this->x_hat_prev + Bobs*this->obs_in_prev;
  xi_hat = x_hat(3,0);
  yaw_hat = x_hat(1,0);
  lat_speed_hat = x_hat(0,0);
  yaw_rate_hat = x_hat(2,0);


  // LTV System:
  if (abs(this->speed_) < 0.5){
    a22 = 0.0;
    a24 = 0.0;
    a42 = 0.0;
    a44 = 0.0;
    this->control_signal = (Izz/(2*lf*Caf))*(-K1*(this->yaw_rate - this->desired_yaw_rate) - K2*(this->heading - this->yaw_ref));
  }
  else {
    a22 = - (2*Caf + 2*Car) / (m*this->speed_);
    a24 = -this->speed_ - (2*Caf*lf - 2*Car*lr) / (m*this->speed_);
    a42 = -(2*lf*Caf-2*lr*Car)/(Izz*this->speed_); 
    a44 = -(2*lf*lf*Caf+2*lr*lr*Car)/(Izz*this->speed_);
    // Compute control signal from control equation:
    this->control_signal = (Izz/(2*lf*Caf))*(-K1*(this->yaw_rate - this->desired_yaw_rate) - K2*(this->heading - this->yaw_ref) - a42*this->lat_speed - a44*this->yaw_rate); //- xi_hat);
  }




  RCLCPP_DEBUG(this->get_logger(), "Pre-clamped u: '%f'", this->control_signal);

  //RCLCPP_INFO(this->get_logger(), "Yaw comparison '%f, %f'", x_hat(1,0), this->heading);

  RCLCPP_INFO(this->get_logger(), "Yaw comparison '%f, %f'", x_hat(1,0), this->heading);
  RCLCPP_INFO(this->get_logger(), "Yaw rate comparison '%f, %f'", yaw_rate_hat, this->yaw_rate);

  RCLCPP_DEBUG(this->get_logger(), "xi_hat: '%f'", xi_hat);

  // Saturation:
  /*if (this->control_signal > this->max_steer_angle*M_PI/180) { 
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Max");
    this->control_signal = this->max_steer_angle*M_PI/180;
  }
  if (this->control_signal < -this->max_steer_angle*M_PI/180) { 
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Min");
    this->control_signal = -this->max_steer_angle*M_PI/180;
  }*/


  // Saturation:
  if (this->control_signal > 0.3) { // FIXME: Don't hard code me
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Max");
    this->control_signal = 0.3;
  }
  if (this->control_signal < -0.3) { // FIXME: Don't hard code me
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Min");
    this->control_signal = -0.3;
  }


  // Store values for next callback after saturation:
  Eigen::Matrix<double, 4, 1> obs_in;
  obs_in << this->control_signal,
            this->lat_speed,
            this->heading,
            this->yaw_rate;        


  this->x_hat_prev = x_hat;
  this->obs_in_prev = obs_in;

  // Publish controller debugging msg:
  if (this->pub_debug_signals){
    blackandgold_msgs::msg::ADRC control_msg;
    control_msg.yaw_des = this->yaw_ref;
    control_msg.yaw_ref = yaw_ref;
    control_msg.yaw = this->heading;
    control_msg.yaw_rate_des = this->desired_yaw_rate;
    control_msg.yaw_rate = yaw_rate;
    control_msg.yaw_accel = this->lat_speed; // FIXME: Incorrect value matching here, change msg
    control_msg.yaw_hat = yaw_hat;
    control_msg.yaw_rate_hat = yaw_rate_hat;
    control_msg.yaw_accel_hat = lat_speed_hat; // FIXME: Incorrect value matching here, change yaw_accel for dy
    control_msg.xi_hat = xi_hat;
    control_msg.lat_error = this->lat_error.data;
    control_msg.lookahead_error = this->lookahead_error.data;
    control_msg.steering = this->control_signal;
    controlDebugSignals_->publish(control_msg);
  }

}

void ADRCLatControl::calculateSteeringCmd() {
  // Convert desired yaw rate to steering angle using kinematic model
  this->steering_cmd.data = this->control_signal * 180.0 / M_PI;
  //this->steering_cmd.data = this->steering_cmd.data * 180.0 / M_PI;
  RCLCPP_DEBUG(this->get_logger(), "Steering cmd: '%f'", this->steering_cmd.data);
}

void ADRCLatControl::setCmdsToZeros() {
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

void ADRCLatControl::publishSteering() {
  RCLCPP_DEBUG(this->get_logger(), "Steering: '%f'", steering_cmd.data);
  this->previous_control_signal = this->control_signal;
  if (!this->get_parameter("mute").as_bool()) pubSteeringCmd_->publish(this->steering_cmd);
}

void ADRCLatControl::publishDebugSignals() {
  // check for too large lookahead error and publish to safety_node
  if (fabs(this->lookahead_error.data) > this->max_lookahead_error) {
    this->error_msg.description =
        "lookahead error too high:" + std::to_string(this->lookahead_error.data);
    this->error_msg.origin = "ADRC_lat_control_node";
    this->error_msg.lifetime = 1;
    this->error_msg.module = "Control";
    this->error_msg.severity = 3;
    this->pubErrorReporter_->publish(this->error_msg);
  }

  // check for too large lateral error
  if (fabs(this->lat_error.data) > this->max_lat_error) {
    this->error_msg.description =
        "<CRITICAL> lateral error too high: " + std::to_string(this->lat_error.data);
    this->error_msg.origin = "ADRC_lat_control_node";
    this->error_msg.lifetime = 10;
    this->error_msg.module = "Control";
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
      this->error_msg.origin = "ADRC_lat_control_node";
      this->error_msg.lifetime = 10;
      this->error_msg.module = "Control";
      this->error_msg.severity = 4;
      this->pubErrorReporter_->publish(this->error_msg);
    }
  }

  pubLookaheadError_->publish(this->lookahead_error);
  pubLatError_->publish(this->lat_error);
}

void ADRCLatControl::receiveOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
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

double ADRCLatControl::quaternionToHeading(const Quaternion& q) {
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

float ADRCLatControl::_dist(float x, float y, float z) {
    return std::sqrt(std::pow(x,2) + std::pow(y,2) + std::pow(z,2));
}

void ADRCLatControl::receiveCurvature(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  this->path_curvatures = *msg;
  this->curvature_ = path_curvatures.data[idx]; 
  std_msgs::msg::Float32 curvature_msg;
  curvature_msg.data = this->curvature_;
  pubCurvature->publish(curvature_msg);
  RCLCPP_DEBUG(this->get_logger(), "Curvature: '%f'", this->curvature_);
}

void ADRCLatControl::calculateCurvatureVelocity(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

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
    this->error_msg.origin = "kin_control_node";
    this->error_msg.lifetime = 10;
    this->error_msg.module = "Control";
    this->error_msg.severity = 5;
    this->pubErrorReporter_->publish(this->error_msg);
  }

}

void ADRCLatControl::receivePath(const nav_msgs::msg::Path::SharedPtr msg) {
  // Determines lookahead distance based on speed and bounds

  this->path_msg = msg;
  double lookahead_distance = std::max(min_la, std::min(max_la, this->speed_ * la_ratio));

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
    this->lat_error.data = -path[0].pose.position.x * norm_y + path[0].pose.position.y * norm_x;
  }

  double alpha = std::atan2(path[idx].pose.position.y, path[idx].pose.position.x);
  this->yaw_ref = alpha + this->heading;
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

std::tuple<int, double> ADRCLatControl::findLookaheadIndex(
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

void ADRCLatControl::receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
  const double kphToMps = 1.0 / 3.6;
  double rear_left = msg->rear_left;
  double rear_right = msg->rear_right;
  this->speed_ =
      (rear_left + rear_right) * 0.5 * kphToMps;  // average wheel speeds (kph) and convert to m/s
}

}  // end namespace control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::ADRCLatControl>());
  rclcpp::shutdown();
  return 0;
}
