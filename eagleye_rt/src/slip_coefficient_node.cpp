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
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::YawrateOffset _yawrate_offset_2nd;
static eagleye_msgs::Heading _heading_interpolate_3rd;

struct SlipCoefficientParameter _slip_coefficient_parameter;
struct SlipCoefficientStatus _slip_coefficient_status;

static double _estimate_coefficient;

bool _is_first_correction_velocity = false;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  _rtklib_nav = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor = *msg;
  if (_is_first_correction_velocity == false && msg->correction_velocity.linear.x > _slip_coefficient_parameter.estimated_velocity_threshold)
  {
    _is_first_correction_velocity = true;
  }
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
  if (_is_first_correction_velocity == false)
  {
    return;
  }

  _imu = *msg;
  slip_coefficient_estimate(_imu, _rtklib_nav, _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_2nd, 
    _heading_interpolate_3rd, _slip_coefficient_parameter, &_slip_coefficient_status, &_estimate_coefficient);

  std::cout << "--- \033[1;34m slip_coefficient \033[m ------------------------------" <<  std::endl;
  std::cout<<"\033[1m estimate_coefficient \033[m " << _estimate_coefficient << std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slip_coefficient");

  ros::NodeHandle nh;

  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  nh.getParam("imu_topic: " , subscribe_imu_topic_name);
  nh.getParam("rtklib_nav_topic", subscribe_rtklib_nav_topic_name);
  nh.getParam("reverse_imu", _slip_coefficient_parameter.reverse_imu);
  nh.getParam("slip_coefficient/estimated_number_min", _slip_coefficient_parameter.estimated_number_min);
  nh.getParam("slip_coefficient/estimated_number_max", _slip_coefficient_parameter.estimated_number_max);
  nh.getParam("slip_coefficient/estimated_velocity_threshold", _slip_coefficient_parameter.estimated_velocity_threshold);
  nh.getParam("slip_coefficient/estimated_yawrate_threshold", _slip_coefficient_parameter.estimated_yawrate_threshold);
  nh.getParam("slip_coefficient/lever_arm", _slip_coefficient_parameter.lever_arm);
  nh.getParam("slip_coefficient/stop_judgment_velocity_threshold", _slip_coefficient_parameter.stop_judgment_velocity_threshold);

  std::cout<< "subscribe_imu_topic_name " << subscribe_imu_topic_name << std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;
  std::cout<< "reverse_imu " << _slip_coefficient_parameter.reverse_imu << std::endl;
  std::cout<< "estimated_number_min " << _slip_coefficient_parameter.estimated_number_min << std::endl;
  std::cout<< "estimated_number_max " << _slip_coefficient_parameter.estimated_number_max << std::endl;
  std::cout<< "estimated_velocity_threshold " << _slip_coefficient_parameter.estimated_velocity_threshold << std::endl;
  std::cout<< "estimated_yawrate_threshold " << _slip_coefficient_parameter.estimated_yawrate_threshold << std::endl;
  std::cout<< "lever_arm " << _slip_coefficient_parameter.lever_arm << std::endl;
  std::cout<< "stop_judgment_velocity_threshold " << _slip_coefficient_parameter.stop_judgment_velocity_threshold << std::endl;

  ros::Subscriber sub1 = nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback);
  ros::Subscriber sub2 = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  ros::Subscriber sub3 = nh.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub4 = nh.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  ros::Subscriber sub5 = nh.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback);
  ros::Subscriber sub6 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);

  ros::spin();

  std::string str;
  nh.getParam("output_dir", str);
  std::ofstream ofs(str, std::ios_base::trunc | std::ios_base::out);
  ofs << "slip_coefficient" << " : " << _estimate_coefficient << std::endl;
  ofs.close();

  return 0;
}
