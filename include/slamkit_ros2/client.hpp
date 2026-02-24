/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */
/*
 *  
 *  Modified by JustASimpleCoder february 23 2026
 *
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "slamkit_ros2/utility.hpp"

using  std::chrono_literals::operator""ms;

class ClientNode : public rclcpp::Node
{
    public:
      ClientNode();
      ~ClientNode() = default;
    
    private:
      void imu_callback(const geometry_msgs::msg::Vector3Stamped::ConstPtr & angle_rad);
      void imu_timer_callback();

      rclcpp::TimerBase::SharedPtr degree_pub_timer_;
      rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr degree_pub_;
      rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr degree_sub_;
      geometry_msgs::msg::Vector3Stamped angle_;
};
