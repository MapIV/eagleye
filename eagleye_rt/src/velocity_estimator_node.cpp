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
 * velocity_estimator_node.cpp
 * Author MapIV Takanose
 */

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static ros::Publisher velocity_pub;
static ros::Publisher velocity_status_pub;

static rtklib_msgs::RtklibNav rtklib_nav_msg;
static nmea_msgs::Gpgga gga_msg;
static sensor_msgs::Imu imu_msg;

VelocityEstimator velocity_estimator;
static geometry_msgs::TwistStamped velocity_msg;

static std::string yaml_file;
static std::string subscribe_imu_topic_name = "imu/data_tf_converted";
static std::string subscribe_rtklib_nav_topic_name = "navsat/rtklib_nav";
static std::string subscribe_gga_topic_name = "navsat/gga";

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav_msg = *msg;
}

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  gga_msg = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg = *msg;

  velocity_estimator.VelocityEstimate(imu_msg, rtklib_nav_msg, gga_msg, &velocity_msg);

  eagleye_msgs::StatusStamped velocity_status;
  velocity_status.header = msg->header;
  velocity_status.status = velocity_estimator.getStatus();
  velocity_status_pub.publish(velocity_status);

  if(velocity_status.status.enabled_status)
  {
    velocity_pub.publish(velocity_msg);
  }

}

void velocity_estimator_node(ros::NodeHandle nh)
{
  nh.getParam("yaml_file",yaml_file);
  velocity_estimator.setParam(yaml_file);

  ros::Subscriber rtklib_sub =
      nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber gga_sub = 
      nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber imu_sub =
      nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());

  velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 1000);
  velocity_status_pub = nh.advertise<eagleye_msgs::StatusStamped>("velocity_status", 1000);

  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_estimator");
  ros::NodeHandle nh;

  velocity_estimator_node(nh);

  return 0;
}