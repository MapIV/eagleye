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
 * height_node.cpp
 * Author MapIV  Takanose
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::Imu _imu;
static nmea_msgs::Gpgga _gga;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::StatusStamped _velocity_status;
static eagleye_msgs::Distance _distance;

static ros::Publisher _pub1, _pub2, _pub3, _pub4, _pub5;
static eagleye_msgs::Height _height;
static eagleye_msgs::Pitching _pitching;
static eagleye_msgs::AccXOffset _acc_x_offset;
static eagleye_msgs::AccXScaleFactor _acc_x_scale_factor;

struct HeightParameter _height_parameter;
struct HeightStatus _height_status;

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

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  _distance = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(_use_can_less_mode && !_velocity_status.status.enabled_status) return;
  
  _imu = *msg;
  _height.header = msg->header;
  _height.header.frame_id = "base_link";
  _pitching.header = msg->header;
  _pitching.header.frame_id = "base_link";
  _acc_x_offset.header = msg->header;
  _acc_x_scale_factor.header = msg->header;
  pitching_estimate(_imu, _gga, _velocity, _distance, _height_parameter, &_height_status,
    &_height, &_pitching, &_acc_x_offset, &_acc_x_scale_factor);
  _pub1.publish(_height);
  _pub2.publish(_pitching);
  _pub3.publish(_acc_x_offset);
  _pub4.publish(_acc_x_scale_factor);

  if(_height_status.flag_reliability)
  {
    _pub5.publish(_gga);
  }

  _height_status.flag_reliability = false;
  _height.status.estimate_status = false;
  _pitching.status.estimate_status = false;
  _acc_x_offset.status.estimate_status = false;
  _acc_x_scale_factor.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height");
  ros::NodeHandle nh;

  std::string subscribe_gga_topic_name = "navsat/gga";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _height_parameter.imu_rate = conf["common"]["imu_rate"].as<double>();
    _height_parameter.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    _height_parameter.moving_judgement_threshold = conf["common"]["moving_judgement_threshold"].as<double>();

    _height_parameter.estimated_minimum_interval = conf["height"]["estimated_minimum_interval"].as<double>();
    _height_parameter.estimated_maximum_interval = conf["height"]["estimated_maximum_interval"].as<double>();
    _height_parameter.update_distance = conf["height"]["update_distance"].as<double>();
    _height_parameter.gnss_receiving_threshold = conf["height"]["gnss_receiving_threshold"].as<double>();
    _height_parameter.outlier_threshold = conf["height"]["outlier_threshold"].as<double>();
    _height_parameter.outlier_ratio_threshold = conf["height"]["outlier_ratio_threshold"].as<double>();
    _height_parameter.moving_average_time = conf["height"]["moving_average_time"].as<double>();

    std::cout << "imu_rate " << _height_parameter.imu_rate << std::endl;
    std::cout << "gnss_rate " << _height_parameter.gnss_rate << std::endl;
    std::cout << "moving_judgement_threshold " << _height_parameter.moving_judgement_threshold << std::endl;

    std::cout << "estimated_minimum_interval " << _height_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << _height_parameter.estimated_maximum_interval << std::endl;
    std::cout << "update_distance " << _height_parameter.update_distance << std::endl;
    std::cout << "gnss_receiving_threshold " << _height_parameter.gnss_receiving_threshold << std::endl;
    std::cout << "outlier_threshold " << _height_parameter.outlier_threshold << std::endl;
    std::cout << "outlier_ratio_threshold " << _height_parameter.outlier_ratio_threshold << std::endl;
    std::cout << "moving_average_time " << _height_parameter.moving_average_time << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mheight Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  ros::Subscriber sub1 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("velocity", 1000, velocity_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe("velocity_status", 1000, velocity_status_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());

  _pub1 = nh.advertise<eagleye_msgs::Height>("height", 1000);
  _pub2 = nh.advertise<eagleye_msgs::Pitching>("pitching", 1000);
  _pub3 = nh.advertise<eagleye_msgs::AccXOffset>("acc_x_offset", 1000);
  _pub4 = nh.advertise<eagleye_msgs::AccXScaleFactor>("acc_x_scale_factor", 1000);
  _pub5 = nh.advertise<nmea_msgs::Gpgga>("navsat/reliability_gga", 1000);

  ros::spin();

  return 0;
}
