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

#ifndef SLAMKIT_ROS2__UTILITY_HPP_
#define SLAMKIT_ROS2__UTILITY_HPP_

// Node names
static inline const char * CLIENT_NODE_NAME = "slamkit_node_client";
static inline const char * IMU_PUB_NODE_NAME = "slamkit_node";

// Topic names node
static inline const char * TOPIC_IMU_RAW = "imu/data_raw";
static inline const char * TOPIC_IMU_FILTERED = "imu/processed_yaw";
static inline const char * TOPIC_MAG_RAW = "imu/mag";

// Topic names client
static inline const char * TOPIC_IMU_ANGLE = "imu/angles_degree";
static inline const char * TOPIC_IMU_RPY = "imu/rpy/filtered";


// parameter string literals
static inline const char * CHANNEL_TYPE_LITERAL = "channel_type";
static inline const char * FRAME_ID_LITERAL = "frame_id";
static inline const char * VENDOR_ID_LITERAL = "usb_venderId_slamkit";
static inline const char * PRODUCT_ID_LITERAL = "usb_productId_slamkit";
static inline const char * INTERFACE_ID_LITERAL = "usb_interfaceId_slamkit";
static inline const char * TX_ENDPOINT_LITERAL = "usb_txEndpoint_slamkit";
static inline const char * RX_ENDPOINT_LITERAL = "usb_rxEndpoint_slamkit";

// Logger string literals
static inline const char * LOGGER_NODE_MAIN = "Slamkit_Main_Node";
static inline const char * LOGGER_CLIENT = "Slamkit_Client_Node";

// math stuff
static inline constexpr double MY_PI = 3.141592654;

// degree to rad conversions
static inline constexpr double DEGREE_TO_RAD = MY_PI / 180.0;
static inline constexpr double RAD_TO_DEGREE = 180.0 / MY_PI;

// imu datasheet factors
static inline constexpr double SHIFT_15_BITS = 32768.00;
static inline constexpr double ACCEl_SENSITIVITY_SCALE_FACTOR = 16384.00;

#endif  // SLAMKIT_ROS2__UTILITY_HPP_
