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
#include "navigation.hpp"

static bool reverse_imu;

static ros::Publisher pub;
static eagleye_msgs::YawrateOffset yawrate_offset;
static eagleye_msgs::AngularVelocityOffset angular_velocity_offset_stop;
static eagleye_msgs::AccXOffset acc_x_offset;
static eagleye_msgs::AccXScaleFactor acc_x_scale_factor;
static sensor_msgs::Imu imu;

static sensor_msgs::Imu correction_imu;


void yawrate_offset_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset.header = msg->header;
  yawrate_offset.yawrate_offset = msg->yawrate_offset;
  yawrate_offset.status = msg->status;
}

void angular_velocity_offset_stop_callback(const eagleye_msgs::AngularVelocityOffset::ConstPtr& msg)
{
  angular_velocity_offset_stop.header = msg->header;
  angular_velocity_offset_stop.rollrate_offset = msg->rollrate_offset;
  angular_velocity_offset_stop.pitchrate_offset = msg->pitchrate_offset;
  angular_velocity_offset_stop.yawrate_offset = msg->yawrate_offset;
  angular_velocity_offset_stop.status = msg->status;
}

void acc_x_offset_callback(const eagleye_msgs::AccXOffset::ConstPtr& msg)
{
  acc_x_offset.header = msg->header;
  acc_x_offset.acc_x_offset = msg->acc_x_offset;
  acc_x_offset.status = msg->status;
}

void acc_x_scale_factor_callback(const eagleye_msgs::AccXScaleFactor::ConstPtr& msg)
{
  acc_x_scale_factor.header = msg->header;
  acc_x_scale_factor.acc_x_scale_factor = msg->acc_x_scale_factor;
  acc_x_scale_factor.status = msg->status;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

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

  if (reverse_imu == false)
  {
    correction_imu.angular_velocity.x = imu.angular_velocity.x + angular_velocity_offset_stop.rollrate_offset;
    correction_imu.angular_velocity.y = imu.angular_velocity.y + angular_velocity_offset_stop.pitchrate_offset;
    correction_imu.angular_velocity.z = -1 * (imu.angular_velocity.z + angular_velocity_offset_stop.yawrate_offset);
  }
  else if (reverse_imu == true)
  {
    correction_imu.angular_velocity.x = imu.angular_velocity.x;
    correction_imu.angular_velocity.y = imu.angular_velocity.y;
    correction_imu.angular_velocity.z = -1 * (-1 * (imu.angular_velocity.z + angular_velocity_offset_stop.yawrate_offset));
  }

  pub.publish(correction_imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "correction_imu");
  ros::NodeHandle n("~");

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/yawrate_offset_2nd", 1000, yawrate_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/eagleye/angular_velocity_offset_stop", 1000, angular_velocity_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("/eagleye/acc_x_offset", 1000, acc_x_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("/eagleye/acc_x_scale_factor", 1000, acc_x_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe("/imu/data_raw", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
<<<<<<< HEAD
  pub = n.advertise<sensor_msgs::Imu>("/imu/data_corrected", 1000);
=======
  pub = n.advertise<sensor_msgs::Imu>("/eagleye/imu/correction_data", 1000);
>>>>>>> 1b9d043b3d64dfcd3f1b41d33bfee3fb3cc72456

  ros::spin();

  return 0;
}
