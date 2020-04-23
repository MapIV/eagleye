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
 * yawrate_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "navigation.hpp"

static geometry_msgs::TwistStamped velocity;
static ros::Publisher pub;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static sensor_msgs::Imu imu;

struct YawrateOffsetStopParameter yawrate_offset_stop_parameter;
struct YawrateOffsetStopStatus yawrate_offset_stop_status;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  velocity.header = msg->header;
  velocity.twist = msg->twist;
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
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop_estimate(velocity, imu, yawrate_offset_stop_parameter, &yawrate_offset_stop_status, &yawrate_offset_stop);
  pub.publish(yawrate_offset_stop);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yawrate_offset_stop");
  ros::NodeHandle n("~");

  n.getParam("/eagleye/reverse_imu", yawrate_offset_stop_parameter.reverse_imu);
  n.getParam("/eagleye/yawrate_offset_stop/stop_judgment_velocity_threshold",yawrate_offset_stop_parameter.stop_judgment_velocity_threshold);
  n.getParam("/eagleye/yawrate_offset_stop/estimated_number",yawrate_offset_stop_parameter.estimated_number);

  std::cout<< "reverse_imu "<<yawrate_offset_stop_parameter.reverse_imu<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<yawrate_offset_stop_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "estimated_number "<<yawrate_offset_stop_parameter.estimated_number<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/can_twist", 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/imu/data_raw", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  pub = n.advertise<eagleye_msgs::YawrateOffset>("/eagleye/yawrate_offset_stop", 1000);

  ros::spin();

  return 0;
}
