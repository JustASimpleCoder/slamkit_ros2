# Copyright 2014 RoboPeak
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#  Modified by JustASimpleCoder February 23, 2026

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
                        "publish_debug_topics": False,
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
        ]
    )
