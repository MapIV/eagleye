// Copyright (c) 2022, Map IV, Inc.
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
 * rolling_node.cpp
 * Author MapIV Takanose
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static ros::Publisher rolling_pub;

static eagleye_msgs::VelocityScaleFactor velocity_scale_factor_msg;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd_msg;
static eagleye_msgs::YawrateOffset yawrate_offset_stop_msg;
static sensor_msgs::Imu imu_msg;

static eagleye_msgs::Rolling rolling_msg;

struct RollingParameter rolling_parameter;
struct RollingStatus rolling_status;

static std::string subscribe_imu_topic_name;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor_msg = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_stop_msg = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_2nd_msg = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg = *msg;
  rolling_estimate(imu_msg, velocity_scale_factor_msg, yawrate_offset_stop_msg, yawrate_offset_2nd_msg,
                   rolling_parameter, &rolling_status, &rolling_msg);
  rolling_pub.publish(rolling_msg);
}

void setParam(ros::NodeHandle nh)
{
  nh.getParam("imu_topic", subscribe_imu_topic_name);
  nh.getParam("reverse_imu", rolling_parameter.reverse_imu);
  nh.getParam("rolling/stop_judgment_velocity_threshold", rolling_parameter.stop_judgment_velocity_threshold);
  nh.getParam("rolling/filter_process_noise", rolling_parameter.filter_process_noise);
  nh.getParam("rolling/filter_observation_noise", rolling_parameter.filter_observation_noise);

  std::cout << "subscribe_imu_topic_name " << subscribe_imu_topic_name << std::endl;
  std::cout << "reverse_imu " << rolling_parameter.reverse_imu << std::endl;
  std::cout << "stop_judgment_velocity_threshold " << rolling_parameter.stop_judgment_velocity_threshold << std::endl;
  std::cout << "filter_process_noise " << rolling_parameter.filter_process_noise << std::endl;
  std::cout << "filter_observation_noise " << rolling_parameter.filter_observation_noise << std::endl;
}

void rolling_node(ros::NodeHandle nh)
{
  setParam(nh);

  ros::Subscriber imu_sub =
      nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber velocity_scale_factor_sub =
      nh.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber yawrate_offset_2nd_sub =
      nh.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber yawrate_offset_stop_sub =
      nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());

  rolling_pub = nh.advertise<eagleye_msgs::Rolling>("rolling", 1000);

  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rolling");
  ros::NodeHandle nh;

  rolling_node(nh);

  return 0;
}