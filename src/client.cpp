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

#include "slamkit_ros2/client.hpp"

ClientNode::ClientNode()
: Node(CLIENT_NODE_NAME),
  angle_{}
{
  degree_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(TOPIC_IMU_ANGLE, 100);
  // degree_pub_timer_ = this->create_wall_timer(
  //   500ms, std::bind(&ClientNode::imu_timer_callback, this)
  // );

  degree_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    TOPIC_IMU_RPY, 10, std::bind(&ClientNode::imu_callback, this, std::placeholders::_1)
  );
}

void ClientNode::imu_timer_callback()
{
  degree_pub_->publish(angle_);
}


void ClientNode::imu_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr & angle_rad)
{
  const double angle_degree_roll = angle_rad->vector.x * 180.0 / MY_PI;
  const double angle_degree_pitch = angle_rad->vector.y * 180.0 / MY_PI;
  const double angle_degree_yaw = angle_rad->vector.z * 180.0 / MY_PI;

  angle_.header.stamp = this->get_clock()->now();

  angle_.header.frame_id = "angle_degree";
  angle_.vector.x = angle_degree_roll;
  angle_.vector.y = angle_degree_pitch;
  angle_.vector.z = angle_degree_yaw;

  degree_pub_->publish(angle_);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClientNode>());
  rclcpp::shutdown();
  return 0;
}
