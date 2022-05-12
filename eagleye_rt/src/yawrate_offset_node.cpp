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
 * yawrate_offset.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::Heading _heading_interpolate;
static sensor_msgs::Imu _imu;
static ros::Publisher _pub;
static eagleye_msgs::YawrateOffset _yawrate_offset;

struct YawrateOffsetParameter _yawrate_offset_parameter;
struct YawrateOffsetStatus _yawrate_offset_status;

bool _is_first_heading= false;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _velocity = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_stop = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate = *msg;

  if (_is_first_heading == false && _heading_interpolate.status.enabled_status == true)
  {
    _is_first_heading = true;
  }
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (_is_first_heading == false)
  {
    return;
  }

  _imu = *msg;
  _yawrate_offset.header = msg->header;
  yawrate_offset_estimate(_velocity, _yawrate_offset_stop, _heading_interpolate, _imu, _yawrate_offset_parameter, &_yawrate_offset_status, &_yawrate_offset);
  _pub.publish(_yawrate_offset);
  _yawrate_offset.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yawrate_offset");
  ros::NodeHandle nh;

  nh.getParam("yawrate_offset/estimated_number_min", _yawrate_offset_parameter.estimated_number_min);
  nh.getParam("yawrate_offset/estimated_coefficient", _yawrate_offset_parameter.estimated_coefficient);
  nh.getParam("yawrate_offset/estimated_velocity_threshold" , _yawrate_offset_parameter.estimated_velocity_threshold);
  nh.getParam("yawrate_offset/outlier_threshold", _yawrate_offset_parameter.outlier_threshold);

  std::cout<< "estimated_number_min: " << _yawrate_offset_parameter.estimated_number_min << std::endl;
  std::cout<< "estimated_coefficient: " << _yawrate_offset_parameter.estimated_coefficient << std::endl;
  std::cout<< "estimated_velocity_threshold: " << _yawrate_offset_parameter.estimated_velocity_threshold << std::endl;
  std::cout<< "outlier_threshold: " << _yawrate_offset_parameter.outlier_threshold << std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "yawrate_offset_1st";
      subscribe_topic_name = "heading_interpolate_1st";
      nh.getParam("yawrate_offset/1st/estimated_number_max", _yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max: " << _yawrate_offset_parameter.estimated_number_max << std::endl;
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "yawrate_offset_2nd";
      subscribe_topic_name = "heading_interpolate_2nd";
      nh.getParam("yawrate_offset/2nd/estimated_number_max", _yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max: " << _yawrate_offset_parameter.estimated_number_max << std::endl;
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

  ros::Subscriber sub1 = nh.subscribe("velocity", 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe(subscribe_topic_name, 1000, heading_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<eagleye_msgs::YawrateOffset>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
