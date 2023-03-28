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
 * slip_coefficient.cpp
 * Author MapIV Takanose
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>

static rtklib_msgs::RtklibNav _rtklib_nav;
static sensor_msgs::Imu _imu;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::StatusStamped _velocity_status;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::YawrateOffset _yawrate_offset_2nd;
static eagleye_msgs::Heading _heading_interpolate_3rd;

struct SlipCoefficientParameter _slip_coefficient_parameter;
struct SlipCoefficientStatus _slip_coefficient_status;

static double _estimate_coefficient;

bool _is_first_correction_velocity = false;
static bool _use_canless_mode;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  _rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _velocity = *msg;
  if (_is_first_correction_velocity == false && msg->twist.linear.x > _slip_coefficient_parameter.moving_judgment_threshold)
  {
    _is_first_correction_velocity = true;
  }
}

void velocity_status_callback(const eagleye_msgs::StatusStamped::ConstPtr& msg)
{
  _velocity_status = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_stop = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_2nd = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_3rd = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (_is_first_correction_velocity == false) return;
  if(_use_canless_mode && !_velocity_status.status.enabled_status) return;

  _imu = *msg;
  slip_coefficient_estimate(_imu, _rtklib_nav, _velocity, _yawrate_offset_stop, _yawrate_offset_2nd, 
    _heading_interpolate_3rd, _slip_coefficient_parameter, &_slip_coefficient_status, &_estimate_coefficient);

  std::cout << "--- \033[1;34m slip_coefficient \033[m ------------------------------" <<  std::endl;
  std::cout<<"\033[1m estimate_coefficient \033[m " << _estimate_coefficient << std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slip_coefficient");

  ros::NodeHandle nh;

  std::string subscribe_rtklib_nav_topic_name = "navsat/rtklib_nav";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_canless_mode = conf["use_canless_mode"].as<bool>();

    subscribe_rtklib_nav_topic_name = conf["rtklib_nav_topic"].as<std::string>();

    _slip_coefficient_parameter.imu_rate = conf["common"]["imu_rate"].as<double>();
    _slip_coefficient_parameter.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    _slip_coefficient_parameter.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();

    _slip_coefficient_parameter.estimated_minimum_interval = conf["slip_coefficient"]["estimated_minimum_interval"].as<double>();
    _slip_coefficient_parameter.estimated_maximum_interval = conf["slip_coefficient"]["estimated_maximum_interval"].as<double>();
    _slip_coefficient_parameter.curve_judgment_threshold = conf["slip_coefficient"]["curve_judgment_threshold"].as<double>();
    _slip_coefficient_parameter.lever_arm = conf["slip_coefficient"]["lever_arm"].as<double>();

    std::cout<< "use_canless_mode " << _use_canless_mode << std::endl;

    std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;

    std::cout << "imu_rate " << _slip_coefficient_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << _slip_coefficient_parameter.stop_judgment_threshold << std::endl;
    std::cout << "moving_judgment_threshold " << _slip_coefficient_parameter.moving_judgment_threshold << std::endl;

    std::cout << "estimated_minimum_interval " << _slip_coefficient_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << _slip_coefficient_parameter.estimated_maximum_interval << std::endl;
    std::cout << "curve_judgment_threshold " << _slip_coefficient_parameter.curve_judgment_threshold << std::endl;
    std::cout << "lever_arm " << _slip_coefficient_parameter.lever_arm << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mslip_coefficient Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  ros::Subscriber sub1 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("velocity", 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("velocity_status", 1000, velocity_status_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());

  ros::spin();

  std::string str;
  nh.getParam("output_dir", str);
  std::ofstream ofs(str, std::ios_base::trunc | std::ios_base::out);
  ofs << "slip_coefficient" << " : " << _estimate_coefficient << std::endl;
  ofs.close();

  return 0;
}
