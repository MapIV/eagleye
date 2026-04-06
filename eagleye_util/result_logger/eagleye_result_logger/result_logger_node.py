#!/usr/bin/env python3

# Copyright (c) 2024, Map IV, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the Map IV, Inc. nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
eagleye_result_logger: Save eagleye estimation results to CSV for regression comparison.

Output files (written to --output-dir, default: current directory):
  fix.csv   - NavSatFix: timestamp_s, latitude, longitude, altitude, status
  pose.csv  - PoseStamped: timestamp_s, x, y, z, qx, qy, qz, qw
"""

import csv
import os
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


class ResultLoggerNode(Node):

    def __init__(self):
        super().__init__('eagleye_result_logger')

        self.declare_parameter('output_dir', '.')
        self.declare_parameter('fix_topic', '/eagleye/fix')
        self.declare_parameter('pose_topic', '/eagleye/eagleye/pose')

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        fix_path = os.path.join(output_dir, f'fix_{timestamp}.csv')
        pose_path = os.path.join(output_dir, f'pose_{timestamp}.csv')

        self._fix_file = open(fix_path, 'w', newline='')
        self._pose_file = open(pose_path, 'w', newline='')

        self._fix_writer = csv.writer(self._fix_file)
        self._pose_writer = csv.writer(self._pose_file)

        self._fix_writer.writerow(['timestamp_s', 'latitude', 'longitude', 'altitude', 'status'])
        self._pose_writer.writerow(['timestamp_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        self._fix_count = 0
        self._pose_count = 0

        self._fix_sub = self.create_subscription(
            NavSatFix, fix_topic, self._fix_callback, 10)
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._pose_callback, 10)

        self.get_logger().info(f'Logging fix  -> {fix_path}')
        self.get_logger().info(f'Logging pose -> {pose_path}')

    def _fix_callback(self, msg: NavSatFix):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._fix_writer.writerow([
            f'{t:.9f}',
            msg.latitude,
            msg.longitude,
            msg.altitude,
            msg.status.status,
        ])
        self._fix_count += 1
        if self._fix_count % 100 == 0:
            self._fix_file.flush()

    def _pose_callback(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        self._pose_writer.writerow([
            f'{t:.9f}',
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
        ])
        self._pose_count += 1
        if self._pose_count % 100 == 0:
            self._pose_file.flush()

    def destroy_node(self):
        self._fix_file.flush()
        self._fix_file.close()
        self._pose_file.flush()
        self._pose_file.close()
        self.get_logger().info(
            f'Saved {self._fix_count} fix rows, {self._pose_count} pose rows.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResultLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
