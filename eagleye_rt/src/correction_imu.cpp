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

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static ros::Publisher _pub;
static eagleye_msgs::YawrateOffset _yawrate_offset;
static eagleye_msgs::AngularVelocityOffset _angular_velocity_offset_stop;
static eagleye_msgs::AccXOffset _acc_x_offset;
static eagleye_msgs::AccXScaleFactor _acc_x_scale_factor;
static sensor_msgs::Imu _imu;

static sensor_msgs::Imu _correction_imu;


void yawrate_offset_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset = *msg;
}

void angular_velocity_offset_stop_callback(const eagleye_msgs::AngularVelocityOffset::ConstPtr& msg)
{
  _angular_velocity_offset_stop = *msg;
}

void acc_x_offset_callback(const eagleye_msgs::AccXOffset::ConstPtr& msg)
{
  _acc_x_offset = *msg;
}

void acc_x_scale_factor_callback(const eagleye_msgs::AccXScaleFactor::ConstPtr& msg)
{
  _acc_x_scale_factor = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu = *msg;
  _correction_imu.header = _imu.header;
  _correction_imu.orientation = _imu.orientation;
  _correction_imu.orientation_covariance = _imu.orientation_covariance;
  _correction_imu.angular_velocity_covariance = _imu.angular_velocity_covariance;
  _correction_imu.linear_acceleration_covariance = _imu.linear_acceleration_covariance;

  if (_acc_x_offset.status.enabled_status && _acc_x_scale_factor.status.enabled_status)
  {
    _correction_imu.linear_acceleration.x = _imu.linear_acceleration.x * _acc_x_scale_factor.acc_x_scale_factor + _acc_x_offset.acc_x_offset;
    _correction_imu.linear_acceleration.y = _imu.linear_acceleration.y;
    _correction_imu.linear_acceleration.z = _imu.linear_acceleration.z;
  }
  else
  {
    _correction_imu.linear_acceleration.x = _imu.linear_acceleration.x;
    _correction_imu.linear_acceleration.y = _imu.linear_acceleration.y;
    _correction_imu.linear_acceleration.z = _imu.linear_acceleration.z;
  }

  _correction_imu.angular_velocity.x = _imu.angular_velocity.x + _angular_velocity_offset_stop.angular_velocity_offset.x;
  _correction_imu.angular_velocity.y = _imu.angular_velocity.y + _angular_velocity_offset_stop.angular_velocity_offset.y;
  _correction_imu.angular_velocity.z = -1 * (_imu.angular_velocity.z + _angular_velocity_offset_stop.angular_velocity_offset.z);

  _pub.publish(_correction_imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "correction_imu");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("angular_velocity_offset_stop", 1000, angular_velocity_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("acc_x_offset", 1000, acc_x_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("acc_x_scale_factor", 1000, acc_x_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<sensor_msgs::Imu>("imu/data_corrected", 1000);

  ros::spin();

  return 0;
}
