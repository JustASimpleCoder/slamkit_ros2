// Copyright (c) 2014, RoboPeak
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// RPlidar ROS Node client test app
// Copyright 2009 - 2014 RoboPeak Team
// http://www.robopeak.com
//
//  Modified by JustASimpleCoder february 23 2026

#ifndef SLAMKIT_ROS2__NODE_HPP_
#define SLAMKIT_ROS2__NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "slamkit_ros2/utility.hpp"


#include "sl_slamkit.h"

using sl::CHANNEL_TYPE_SERIALPORT;
using sl::sl_imu_raw_data_t;
using sl::CHANNEL_TYPE_TCP;
using sl::CHANNEL_TYPE_UDP;
using sl::CHANNEL_TYPE_USB;
using sl::ISlamkitDriver;
using sl::createSlamkitDriver;
using sl::createUSBChannel;

using  std::chrono_literals::operator""ms;

static sl_u32 last_ts_ms = 0;
static sl_u32 processed_last_ts_ms = 0;

class ImuPub : public rclcpp::Node
{
public:
  ImuPub();
  ~ImuPub() = default;

  void imu_publish(const sl_imu_raw_data_t & imu_data, const std::string & frame_id);
  void imu_processed_publish(const sl_slamkit_read_imu_processed_response_t & PImu_respc);

private:
  rclcpp::TimerBase::SharedPtr imu_pub_timer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_processed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  sensor_msgs::msg::Imu imu_msg_;
  geometry_msgs::msg::Vector3Stamped imu_processed_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;

  std::string channel_type_;

  int usb_venderId_slamkit_;
  int usb_productId_slamkit_;
  int usb_interfaceId_slamkit_;
  int usb_txEndpoint_slamkit_;
  int usb_rxEndpoint_slamkit_;
  std::string frame_id_;
};

#endif  // SLAMKIT_ROS2__NODE_HPP_
