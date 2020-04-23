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
#include "coordinate.hpp"
#include "navigation.hpp"

static eagleye_msgs::Position enu_absolute_pos;
static geometry_msgs::Vector3Stamped enu_absolute_vel;

static eagleye_msgs::Position enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix eagleye_fix;
static ros::Publisher pub1;
static ros::Publisher pub2;

struct PositionInterpolateParameter position_interpolate_parameter;
struct PositionInterpolateStatus position_interpolate_status;


void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_absolute_vel.header = msg->header;
  enu_absolute_vel.vector = msg->vector;
  enu_absolute_pos_interpolate.header = msg->header;
  eagleye_fix.header = msg->header;
  position_interpolate_estimate(enu_absolute_pos,enu_absolute_vel,position_interpolate_parameter,&position_interpolate_status,&enu_absolute_pos_interpolate,&eagleye_fix);
  pub1.publish(enu_absolute_pos_interpolate);
  pub2.publish(eagleye_fix);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_interpolate");
  ros::NodeHandle n;

  n.getParam("/eagleye/position_interpolate/altitude_estimate",position_interpolate_parameter.altitude_estimate);
  n.getParam("/eagleye/position_interpolate/number_buffer_max", position_interpolate_parameter.number_buffer_max);
  std::cout<< "altitude_estimate "<<position_interpolate_parameter.altitude_estimate<<std::endl;
  std::cout<< "number_buffer_max "<<position_interpolate_parameter.number_buffer_max<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  pub1 = n.advertise<eagleye_msgs::Position>("/eagleye/enu_absolute_pos_interpolate", 1000);
  pub2 = n.advertise<sensor_msgs::NavSatFix>("/eagleye/fix", 1000);

  ros::spin();

  return 0;
}
