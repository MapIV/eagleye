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
 * smoothing.cpp
 * Author MapIV Takanose
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static rtklib_msgs::RtklibNav _rtklib_nav;
static eagleye_msgs::Position _enu_absolute_pos, _gnss_smooth_pos_enu;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::StatusStamped _velocity_status;
static ros::Publisher _pub;

struct SmoothingParameter _smoothing_parameter;
struct SmoothingStatus _smoothing_status;

static bool _use_canless_mode;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::StatusStamped::ConstPtr& msg)
{
  _velocity_status = *msg;
}

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  if(_use_canless_mode && !_velocity_status.status.enabled_status) return;

  _rtklib_nav = *msg;
  _gnss_smooth_pos_enu.header = msg->header;
  _gnss_smooth_pos_enu.header.frame_id = "base_link";
  smoothing_estimate(_rtklib_nav, _velocity, _smoothing_parameter, &_smoothing_status, &_gnss_smooth_pos_enu);
  _pub.publish(_gnss_smooth_pos_enu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smoothing");
  ros::NodeHandle nh;

  std::string subscribe_rtklib_nav_topic_name = "navsat/rtklib_nav";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_canless_mode = conf["use_canless_mode"].as<bool>();

    _smoothing_parameter.ecef_base_pos_x = conf["ecef_base_pos"]["x"].as<double>();
    _smoothing_parameter.ecef_base_pos_y = conf["ecef_base_pos"]["y"].as<double>();
    _smoothing_parameter.ecef_base_pos_z = conf["ecef_base_pos"]["z"].as<double>();

    _smoothing_parameter.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    _smoothing_parameter.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    _smoothing_parameter.moving_average_time = conf["smoothing"]["moving_average_time"].as<double>();
    _smoothing_parameter.moving_ratio_threshold = conf["smoothing"]["moving_ratio_threshold"].as<double>();

    std::cout<< "use_canless_mode " << _use_canless_mode << std::endl;

    std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;

    std::cout<< "ecef_base_pos_x " << _smoothing_parameter.ecef_base_pos_x << std::endl;
    std::cout<< "ecef_base_pos_y " << _smoothing_parameter.ecef_base_pos_y << std::endl;
    std::cout<< "ecef_base_pos_z " << _smoothing_parameter.ecef_base_pos_z << std::endl;

    std::cout << "gnss_rate " << _smoothing_parameter.gnss_rate << std::endl;
    std::cout << "moving_judgment_threshold " << _smoothing_parameter.moving_judgment_threshold << std::endl;
    std::cout << "moving_average_time " << _smoothing_parameter.moving_average_time << std::endl;
    std::cout << "moving_ratio_threshold " << _smoothing_parameter.moving_ratio_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31msmoothing Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  ros::Subscriber sub1 = nh.subscribe("velocity", 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("velocity_status", 1000, velocity_status_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());

  _pub = nh.advertise<eagleye_msgs::Position>("gnss_smooth_pos_enu", 1000);

  ros::spin();

  return 0;
}
