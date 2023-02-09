// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * correction_imu.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset;
static eagleye_msgs::msg::AngularVelocityOffset angular_velocity_offset_stop;
static eagleye_msgs::msg::AccXOffset acc_x_offset;
static eagleye_msgs::msg::AccXScaleFactor acc_x_scale_factor;
static sensor_msgs::msg::Imu imu;

static sensor_msgs::msg::Imu correction_imu;


void yaw_rate_offset_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset = *msg;
}

void angular_velocity_offset_stop_callback(const eagleye_msgs::msg::AngularVelocityOffset::ConstSharedPtr msg)
{
  angular_velocity_offset_stop = *msg;
}

void acc_x_offset_callback(const eagleye_msgs::msg::AccXOffset::ConstSharedPtr msg)
{
  acc_x_offset = *msg;
}

void acc_x_scale_factor_callback(const eagleye_msgs::msg::AccXScaleFactor::ConstSharedPtr msg)
{
  acc_x_scale_factor = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu = *msg;

  correction_imu.header = imu.header;
  correction_imu.orientation = imu.orientation;
  correction_imu.orientation_covariance = imu.orientation_covariance;
  correction_imu.angular_velocity_covariance = imu.angular_velocity_covariance;
  correction_imu.linear_acceleration_covariance = imu.linear_acceleration_covariance;

  if (acc_x_offset.status.enabled_status == true && acc_x_scale_factor.status.enabled_status)
  {
    correction_imu.linear_acceleration.x = imu.linear_acceleration.x * acc_x_scale_factor.acc_x_scale_factor + acc_x_offset.acc_x_offset;
    correction_imu.linear_acceleration.y = imu.linear_acceleration.y;
    correction_imu.linear_acceleration.z = imu.linear_acceleration.z;
  }
  else
  {
    correction_imu.linear_acceleration.x = imu.linear_acceleration.x;
    correction_imu.linear_acceleration.y = imu.linear_acceleration.y;
    correction_imu.linear_acceleration.z = imu.linear_acceleration.z;
  }

  correction_imu.angular_velocity.x = imu.angular_velocity.x + angular_velocity_offset_stop.angular_velocity_offset.x;
  correction_imu.angular_velocity.y = imu.angular_velocity.y + angular_velocity_offset_stop.angular_velocity_offset.y;
  correction_imu.angular_velocity.z = -1 * (imu.angular_velocity.z + angular_velocity_offset_stop.angular_velocity_offset.z);

  pub->publish(correction_imu);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_correction_imu");

  auto sub1 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", rclcpp::QoS(10), yaw_rate_offset_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<eagleye_msgs::msg::AngularVelocityOffset>("angular_velocity_offset_stop", rclcpp::QoS(10), angular_velocity_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::AccXOffset>("acc_x_offset", rclcpp::QoS(10), acc_x_offset_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::AccXScaleFactor>("acc_x_scale_factor", rclcpp::QoS(10), acc_x_scale_factor_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_corrected", rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
