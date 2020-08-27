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

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::Position enu_absolute_pos,gnss_smooth_pos_enu;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static ros::Publisher pub;

struct SmoothingParameter smoothing_parameter;
struct SmoothingStatus smoothing_status;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
  gnss_smooth_pos_enu.header = msg->header;
  gnss_smooth_pos_enu.header.frame_id = "enu";
  smoothing_estimate(rtklib_nav,velocity_scale_factor,smoothing_parameter,&smoothing_status,&gnss_smooth_pos_enu);
  pub.publish(gnss_smooth_pos_enu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smoothing");
  ros::NodeHandle n;

  n.getParam("/eagleye/position/ecef_base_pos_x",smoothing_parameter.ecef_base_pos_x);
  n.getParam("/eagleye/position/ecef_base_pos_y",smoothing_parameter.ecef_base_pos_y);
  n.getParam("/eagleye/position/ecef_base_pos_z",smoothing_parameter.ecef_base_pos_z);
  n.getParam("/eagleye/smoothing/estimated_number_max",smoothing_parameter.estimated_number_max);
  n.getParam("/eagleye/smoothing/estimated_velocity_threshold",smoothing_parameter.estimated_velocity_threshold);
  n.getParam("/eagleye/smoothing/estimated_threshold",smoothing_parameter.estimated_threshold);

  std::cout<< "ecef_base_pos_x "<<smoothing_parameter.ecef_base_pos_x<<std::endl;
  std::cout<< "ecef_base_pos_y "<<smoothing_parameter.ecef_base_pos_y<<std::endl;
  std::cout<< "ecef_base_pos_z "<<smoothing_parameter.ecef_base_pos_z<<std::endl;
  std::cout<< "estimated_number_max "<<smoothing_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<smoothing_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_threshold "<<smoothing_parameter.estimated_threshold<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());

  pub = n.advertise<eagleye_msgs::Position>("/eagleye/gnss_smooth_pos_enu", 1000);

  ros::spin();

  return 0;
}
