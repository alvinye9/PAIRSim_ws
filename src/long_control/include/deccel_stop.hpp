#ifndef DECCEL_STOP_HPP
#define DECCEL_STOP_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"


#include "PID.hpp"

namespace control 

class DeccelStop : public rclcpp::Node
{
  public:
    DeccelStop();


  private:
    void paramUpdateCallback();
    void receiveSpeed(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr param_timer_;
    
    dobule speed_ 
    

    
