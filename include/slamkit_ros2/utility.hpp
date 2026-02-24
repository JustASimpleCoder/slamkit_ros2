// Copyright 2014 RoboPeak
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//  Modified by JustASimpleCoder February 23, 2026

#ifndef SLAMKIT_ROS2__UTILITY_HPP_
#define SLAMKIT_ROS2__UTILITY_HPP_

// Node names
inline const char * CLIENT_NODE_NAME = "slamkit_node_client";
inline const char * IMU_PUB_NODE_NAME = "slamkit_node";

// Topic names node
inline const char * TOPIC_IMU_RAW = "imu/data_raw";
inline const char * TOPIC_IMU_FILTERED = "imu/processed_yaw";
inline const char * TOPIC_MAG_RAW = "imu/mag";

// Topic names client
inline const char * TOPIC_IMU_ANGLE = "imu/angles_degree";
inline const char * TOPIC_IMU_RPY = "imu/rpy/filtered";


// parameter string literals
inline const char * LIRERAL_CHANNEL_TYPE = "channel_type";
inline const char * LITERAL_FRAME_ID = "frame_id";
inline const char * LITERAL_VENDOR_ID = "usb_venderId_slamkit";
inline const char * LITERAL_PRODUCT_ID = "usb_productId_slamkit";
inline const char * LITERAL_INTERFACE_ID = "usb_interfaceId_slamkit";
inline const char * LITERAL_TX_ENDPOINT = "usb_txEndpoint_slamkit";
inline const char * LITERAL_RX_ENDPOINT = "usb_rxEndpoint_slamkit";

// Logger string literals
inline const char * LOGGER_NODE_MAIN = "Slamkit_Main_Node";
inline const char * LOGGER_CLIENT = "Slamkit_Client_Node";

// framde ID for ROS MSGS
inline const char * FRAMDE_ID_IMU = "imu";
inline const char * FRAMDE_ID_IMU_PROCESSED = "imu_processed";
inline const char * FRAMDE_ID_MAG = "magentic";
inline const char * FRAME_ID_ANGLE = "angle_degree";

// math stuff
inline constexpr double MY_PI = 3.141592654;

// degree to rad conversions
inline constexpr double DEGREE_TO_RAD = MY_PI / 180.0;
inline constexpr double RAD_TO_DEGREE = 180.0 / MY_PI;

// imu datasheet factors
inline constexpr double SHIFT_15_BITS = 32768.0;
inline constexpr double ACCEl_SENSITIVITY_SCALE_FACTOR = 16384.0;

// standard gracity constant
inline constexpr double STD_GRAVITY = 9.8;

#endif  // SLAMKIT_ROS2__UTILITY_HPP_
