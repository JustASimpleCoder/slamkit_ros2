# Copyright (c) 2014, RoboPeak
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# RPlidar ROS Node client test app
# Copyright 2009 - 2014 RoboPeak Team
# http://www.robopeak.com
#
# Modified by JustASimpleCoder february 23 2026
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("slamkit_ros2"), "rviz", "imu_display.rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="slamkit_ros2",
                executable="slamkitNode",
                name="slamkitNode",
                output="screen",
                parameters=[
                    {
                        "channel_type": "usb",
                        "frame_id": "imu",
                        "usb_venderId_slamkit": 64719,
                        "usb_productId_slamkit": 61696,
                        "usb_interfaceId_slamkit": 3,
                        "usb_txEndpoint_slamkit": 5,
                        "usb_rxEndpoint_slamkit": 5,
                    }
                ],
            ),
            Node(
                package="imu_complementary_filter",
                executable="complementary_filter_node",
                name="complementary_filter_node",
                output="screen",
                parameters=[
                    {
                        "publish_debug_topics": True,
                        "gain_acc": 0.01,
                    }
                ],
            ),
            Node(
                package="slamkit_ros2",
                executable="slamkitNodeClient",
                name="slamkitNodeClient",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0.1',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'imu_link',
                ],
            ),
        ]
    )
