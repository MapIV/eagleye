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
 * heading.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static nmea_msgs::Gpgga _gga;
static sensor_msgs::Imu _imu;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::StatusStamped _velocity_status;
static eagleye_msgs::Distance _distance;
static eagleye_msgs::YawrateOffset _yaw_rate_offset_stop;
static eagleye_msgs::YawrateOffset _yaw_rate_offset;
static eagleye_msgs::SlipAngle _slip_angle;
static eagleye_msgs::Heading _heading_interpolate;

static ros::Publisher _pub;
static eagleye_msgs::Heading _heading;

struct RtkHeadingParameter _heading_parameter;
struct RtkHeadingStatus _heading_status;

static bool _use_can_less_mode;

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  _gga = *msg;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::StatusStamped::ConstPtr& msg)
{
  _velocity_status = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset_stop = *msg;
}

void yaw_rate_offset_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset = *msg;
}

void slip_angle_callback(const eagleye_msgs::SlipAngle::ConstPtr& msg)
{
  _slip_angle = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate = *msg;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  _distance = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(_use_can_less_mode && !_velocity_status.status.enabled_status) return;
  
  _imu = *msg;
  _heading.header = msg->header;
  _heading.header.frame_id = "base_link";
  rtk_heading_estimate(_gga, _imu, _velocity, _distance, _yaw_rate_offset_stop, _yaw_rate_offset,
    _slip_angle, _heading_interpolate, _heading_parameter, &_heading_status, &_heading);

  if (_heading.status.estimate_status)
  {
    _pub.publish(_heading);
  }
  _heading.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtk_heading");
  ros::NodeHandle nh;

  std::string subscribe_gga_topic_name = "navsat/gga";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_can_less_mode = conf["use_can_less_mode"].as<bool>();

    _heading_parameter.imu_rate = conf["common"]["imu_rate"].as<double>();
    _heading_parameter.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    _heading_parameter.stop_judgement_threshold = conf["common"]["stop_judgement_threshold"].as<double>();
    _heading_parameter.slow_judgement_threshold = conf["common"]["slow_judgement_threshold"].as<double>();

    _heading_parameter.update_distance = conf["rtk_heading"]["update_distance"].as<double>();
    _heading_parameter.estimated_minimum_interval = conf["rtk_heading"]["estimated_minimum_interval"].as<double>();
    _heading_parameter.estimated_maximum_interval = conf["rtk_heading"]["estimated_maximum_interval"].as<double>();
    _heading_parameter.gnss_receiving_threshold = conf["rtk_heading"]["gnss_receiving_threshold"].as<double>();
    _heading_parameter.outlier_threshold = conf["rtk_heading"]["outlier_threshold"].as<double>();
    _heading_parameter.outlier_ratio_threshold = conf["rtk_heading"]["outlier_ratio_threshold"].as<double>();
    _heading_parameter.curve_judgement_threshold = conf["rtk_heading"]["curve_judgement_threshold"].as<double>();

    std::cout<< "use_can_less_mode " << _use_can_less_mode << std::endl;

    std::cout<< "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;

    std::cout << "imu_rate " << _heading_parameter.imu_rate << std::endl;
    std::cout << "gnss_rate " << _heading_parameter.gnss_rate << std::endl;
    std::cout << "stop_judgement_threshold " << _heading_parameter.stop_judgement_threshold << std::endl;
    std::cout << "slow_judgement_threshold " << _heading_parameter.slow_judgement_threshold << std::endl;

    std::cout << "update_distance " << _heading_parameter.update_distance << std::endl;
    std::cout << "estimated_minimum_interval " << _heading_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << _heading_parameter.estimated_maximum_interval << std::endl;
    std::cout << "gnss_receiving_threshold " << _heading_parameter.gnss_receiving_threshold << std::endl;
    std::cout << "outlier_threshold " << _heading_parameter.outlier_threshold << std::endl;
    std::cout << "outlier_ratio_threshold " << _heading_parameter.outlier_ratio_threshold << std::endl;
    std::cout << "curve_judgement_threshold " << _heading_parameter.curve_judgement_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mrtk_heading Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";
  std::string subscribe_topic_name2 = "/subscribe_topic_name2/invalid";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "heading_1st";
      subscribe_topic_name = "yaw_rate_offset_stop";
      subscribe_topic_name2 = "heading_interpolate_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "heading_2nd";
      subscribe_topic_name = "yaw_rate_offset_1st";
      subscribe_topic_name2 = "heading_interpolate_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "heading_3rd";
      subscribe_topic_name = "yaw_rate_offset_2nd";
      subscribe_topic_name2 = "heading_interpolate_3rd";
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
  ros::Subscriber sub2 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("velocity", 1000, velocity_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("velocity_status", 1000, velocity_status_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe("yaw_rate_offset_stop", 1000, yaw_rate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe(subscribe_topic_name, 1000, yaw_rate_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = nh.subscribe("slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub8 = nh.subscribe(subscribe_topic_name2, 1000, heading_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub9 = nh.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());

  _pub = nh.advertise<eagleye_msgs::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
