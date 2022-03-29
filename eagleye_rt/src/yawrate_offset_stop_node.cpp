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
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static geometry_msgs::TwistStamped _velocity;
static ros::Publisher _pub;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static sensor_msgs::Imu _imu;

struct YawrateOffsetStopParameter _yawrate_offset_stop_parameter;
struct YawrateOffsetStopStatus _yawrate_offset_stop_status;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _velocity = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu = *msg;
  _yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop_estimate(_velocity, _imu, _yawrate_offset_stop_parameter, &_yawrate_offset_stop_status, &_yawrate_offset_stop);
  _pub.publish(_yawrate_offset_stop);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yawrate_offset_stop");
  ros::NodeHandle nh;

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";

  nh.getParam("twist_topic", subscribe_twist_topic_name);
  nh.getParam("imu_topic", subscribe_imu_topic_name);
  nh.getParam("reverse_imu" , _yawrate_offset_stop_parameter.reverse_imu);
  nh.getParam("yawrate_offset_stop/stop_judgment_velocity_threshold",  _yawrate_offset_stop_parameter.stop_judgment_velocity_threshold);
  nh.getParam("yawrate_offset_stop/estimated_number" , _yawrate_offset_stop_parameter.estimated_number);
  nh.getParam("yawrate_offset_stop/outlier_threshold" , _yawrate_offset_stop_parameter.outlier_threshold);

  std::cout<< "subscribe_twist_topic_name: " << subscribe_twist_topic_name << std::endl;
  std::cout<< "subscribe_imu_topic_name: " << subscribe_imu_topic_name << std::endl;
  std::cout<< "reverse_imu: " << _yawrate_offset_stop_parameter.reverse_imu << std::endl;
  std::cout<< "stop_judgment_velocity_threshold: " << _yawrate_offset_stop_parameter.stop_judgment_velocity_threshold << std::endl;
  std::cout<< "estimated_number: " << _yawrate_offset_stop_parameter.estimated_number << std::endl;
  std::cout<< "outlier_threshold: " << _yawrate_offset_stop_parameter.outlier_threshold << std::endl;

  ros::Subscriber sub1 = nh.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<eagleye_msgs::YawrateOffset>("yawrate_offset_stop", 1000);

  ros::spin();

  return 0;
}
