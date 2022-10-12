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
 * position_interpolate.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static eagleye_msgs::Position _enu_absolute_pos;
static geometry_msgs::Vector3Stamped _enu_vel;
static eagleye_msgs::Height _height;
static eagleye_msgs::Position _gnss_smooth_pos;
static nmea_msgs::Gpgga _gga;

static eagleye_msgs::Position _enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix _eagleye_fix;
static ros::Publisher _pub1;
static ros::Publisher _pub2;

struct PositionInterpolateParameter _position_interpolate_parameter;
struct PositionInterpolateStatus _position_interpolate_status;

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  _gga = *msg;
}

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _enu_absolute_pos = *msg;
}

void gnss_smooth_pos_enu_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _gnss_smooth_pos = *msg;
}

void height_callback(const eagleye_msgs::Height::ConstPtr& msg)
{
  _height = *msg;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  _enu_vel = *msg;
  _enu_absolute_pos_interpolate.header = msg->header;
  _enu_absolute_pos_interpolate.header.frame_id = "base_link";
  _eagleye_fix.header = msg->header;
  _eagleye_fix.header.frame_id = "gnss";
  position_interpolate_estimate(_enu_absolute_pos, _enu_vel, _gnss_smooth_pos, _height, _position_interpolate_parameter, &_position_interpolate_status, &_enu_absolute_pos_interpolate, &_eagleye_fix);
  if(_enu_absolute_pos.status.enabled_status)
  {
    _pub1.publish(_enu_absolute_pos_interpolate);
    _pub2.publish(_eagleye_fix);
  }
  else if (_gga.header.stamp.toSec() != 0)
  {
    sensor_msgs::NavSatFix fix;
    fix.header = _gga.header;
    fix.latitude = _gga.lat;
    fix.longitude = _gga.lon;
    fix.altitude = _gga.alt + _gga.undulation;
    // TODO(Map IV): temporary covariance value
    fix.position_covariance[0] = 1.5 * 1.5; // [m^2]
    fix.position_covariance[4] = 1.5 * 1.5; // [m^2]
    fix.position_covariance[8] = 1.5 * 1.5; // [m^2]
    _pub2.publish(fix);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_interpolate");
  ros::NodeHandle nh;

  std::string subscribe_gga_topic_name = "navsat/gga";

  nh.getParam("gga_topic",subscribe_gga_topic_name);
  nh.getParam("position_interpolate/number_buffer_max", _position_interpolate_parameter.number_buffer_max);
  nh.getParam("position_interpolate/stop_judgment_velocity_threshold", _position_interpolate_parameter.stop_judgment_velocity_threshold);
  std::cout<< "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;
  std::cout<< "number_buffer_max " << _position_interpolate_parameter.number_buffer_max << std::endl;
  std::cout<< "stop_judgment_velocity_threshold " << _position_interpolate_parameter.stop_judgment_velocity_threshold << std::endl;

  ros::Subscriber sub1 = nh.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("gnss_smooth_pos_enu", 1000, gnss_smooth_pos_enu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("height", 1000, height_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  _pub1 = nh.advertise<eagleye_msgs::Position>("enu_absolute_pos_interpolate", 1000);
  _pub2 = nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);

  ros::spin();

  return 0;
}
