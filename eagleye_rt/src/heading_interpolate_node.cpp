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
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::StatusStamped _velocity_status;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::YawrateOffset _yawrate_offset;
static eagleye_msgs::Heading _heading;
static eagleye_msgs::SlipAngle _slip_angle;

static ros::Publisher _pub;
static eagleye_msgs::Heading _heading_interpolate;

struct HeadingInterpolateParameter _heading_interpolate_parameter;
struct HeadingInterpolateStatus _heading_interpolate_status;

static bool _use_canless_mode;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::StatusStamped::ConstPtr& msg)
{
  _velocity_status = *msg;
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

  if(_use_canless_mode && !_velocity_status.status.enabled_status) return;

  _imu = *msg;
  _heading_interpolate.header = msg->header;
  _heading_interpolate.header.frame_id = "base_link";
  heading_interpolate_estimate(_imu, _velocity, _yawrate_offset_stop, _yawrate_offset, _heading, _slip_angle,
    _heading_interpolate_parameter, &_heading_interpolate_status, &_heading_interpolate);
  _pub.publish(_heading_interpolate);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading_interpolate");
  ros::NodeHandle nh;

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _heading_interpolate_parameter.imu_rate = conf["common"]["imu_rate"].as<double>();
    _heading_interpolate_parameter.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    _heading_interpolate_parameter.sync_search_period = conf["heading_interpolate"]["sync_search_period"].as<double>();
    _heading_interpolate_parameter.proc_noise = conf["heading_interpolate"]["proc_noise"].as<double>();

    std::cout << "imu_rate " << _heading_interpolate_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << _heading_interpolate_parameter.stop_judgment_threshold << std::endl;
    std::cout << "sync_search_period " << _heading_interpolate_parameter.sync_search_period << std::endl;
    std::cout << "proc_noise " << _heading_interpolate_parameter.proc_noise << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mheading_interpolate Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

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

  ros::Subscriber sub1 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("velocity", 1000, velocity_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("velocity_status", 1000, velocity_status_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe(subscribe_topic_name_1, 1000, yawrate_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe(subscribe_topic_name_2, 1000, heading_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = nh.subscribe("slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<eagleye_msgs::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
