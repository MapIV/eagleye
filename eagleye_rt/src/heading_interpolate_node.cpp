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
 * heading_interpolate.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::Imu _imu;
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::YawrateOffset _yawrate_offset;
static eagleye_msgs::Heading _heading;
static eagleye_msgs::SlipAngle _slip_angle;

static ros::Publisher _pub;
static eagleye_msgs::Heading _heading_interpolate;

struct HeadingInterpolateParameter _heading_interpolate_parameter;
struct HeadingInterpolateStatus _heading_interpolate_status;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_stop = *msg;
}

void yawrate_offset_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset = *msg;
}

void heading_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading = *msg;
}

void slip_angle_callback(const eagleye_msgs::SlipAngle::ConstPtr& msg)
{
  _slip_angle = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu = *msg;
  _heading_interpolate.header = msg->header;
  _heading_interpolate.header.frame_id = "base_link";
  heading_interpolate_estimate(_imu, _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset, _heading, _slip_angle,
    _heading_interpolate_parameter, &_heading_interpolate_status, &_heading_interpolate);
  _pub.publish(_heading_interpolate);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading_interpolate");
  ros::NodeHandle nh;

  std::string subscribe_imu_topic_name = "/imu/data_raw";

  nh.getParam("imu_topic: ", subscribe_imu_topic_name);
  nh.getParam("reverse_imu: ", _heading_interpolate_parameter.reverse_imu);
  nh.getParam("heading_interpolate/stop_judgment_velocity_threshold: ", _heading_interpolate_parameter.stop_judgment_velocity_threshold);
  nh.getParam("heading_interpolate/number_buffer_max: ", _heading_interpolate_parameter.number_buffer_max);
  std::cout<< "subscribe_imu_topic_name: " << subscribe_imu_topic_name << std::endl;
  std::cout<< "reverse_imu: " << _heading_interpolate_parameter.reverse_imu << std::endl;
  std::cout<< "stop_judgment_velocity_threshold: " << _heading_interpolate_parameter.stop_judgment_velocity_threshold << std::endl;
  std::cout<< "number_buffer_max: " << _heading_interpolate_parameter.number_buffer_max << std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name_1 = "/subscribe_topic_name/invalid_1";
  std::string subscribe_topic_name_2 = "/subscribe_topic_name/invalid_2";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "heading_interpolate_1st";
      subscribe_topic_name_1 = "yawrate_offset_stop";
      subscribe_topic_name_2 = "heading_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "heading_interpolate_2nd";
      subscribe_topic_name_1 = "yawrate_offset_1st";
      subscribe_topic_name_2 = "heading_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "heading_interpolate_3rd";
      subscribe_topic_name_1 = "yawrate_offset_2nd";
      subscribe_topic_name_2 = "heading_3rd";
    }
    else
    {
      ROS_ERROR("Invalid argument");
      ros::shutdown();
    }
  }
  else
  {
    ROS_ERROR("No arguments");
    ros::shutdown();
  }

  ros::Subscriber sub1 = nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe(subscribe_topic_name_1, 1000, yawrate_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe(subscribe_topic_name_2, 1000, heading_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe("slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<eagleye_msgs::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
