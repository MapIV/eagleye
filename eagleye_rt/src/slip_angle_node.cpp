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
 * slip_angle.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::Imu _imu;
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::YawrateOffset _yawrate_offset_2nd;

static ros::Publisher _pub;
static eagleye_msgs::SlipAngle _slip_angle;

struct SlipangleParameter _slip_angle_parameter;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_stop = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_2nd = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu = *msg;
  _slip_angle.header = msg->header;
  _slip_angle.header.frame_id = "base_link";
  slip_angle_estimate(_imu, _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_2nd, _slip_angle_parameter, &_slip_angle);
  _pub.publish(_slip_angle);
  _slip_angle.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slip_angle");

  ros::NodeHandle nh;

  std::string subscribe_imu_topic_name = "/imu/data_raw";

  nh.getParam("imu_topic" , subscribe_imu_topic_name);
  nh.getParam("reverse_imu", _slip_angle_parameter.reverse_imu);
  nh.getParam("slip_angle/manual_coefficient", _slip_angle_parameter.manual_coefficient);
  nh.getParam("slip_angle/stop_judgment_velocity_threshold", _slip_angle_parameter.stop_judgment_velocity_threshold);
  std::cout<< "subscribe_imu_topic_name " << subscribe_imu_topic_name << std::endl;
  std::cout<< "reverse_imu " << _slip_angle_parameter.reverse_imu << std::endl;
  std::cout<< "manual_coefficient " << _slip_angle_parameter.manual_coefficient << std::endl;
  std::cout<< "stop_judgment_velocity_threshold " << _slip_angle_parameter.stop_judgment_velocity_threshold << std::endl;

  ros::Subscriber sub1 = nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<eagleye_msgs::SlipAngle>("slip_angle", 1000);

  ros::spin();

  return 0;
}
