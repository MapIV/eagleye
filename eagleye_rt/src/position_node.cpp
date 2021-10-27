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
 * position.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::Distance distance;
static eagleye_msgs::Heading heading_interpolate_3rd;
static eagleye_msgs::Position enu_absolute_pos;
static geometry_msgs::Vector3Stamped enu_vel;
static ros::Publisher pub;

struct PositionParameter position_parameter;
struct PositionStatus position_status;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.header.frame_id = "base_link";
  position_estimate(rtklib_nav, velocity_scale_factor, distance, heading_interpolate_3rd, enu_vel, position_parameter, &position_status, &enu_absolute_pos);
  if(enu_absolute_pos.status.estimate_status == true)
  {
    pub.publish(enu_absolute_pos);
  }
  enu_absolute_pos.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position");
  ros::NodeHandle n;

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  n.getParam("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("position/estimated_distance",position_parameter.estimated_distance);
  n.getParam("position/separation_distance",position_parameter.separation_distance);
  n.getParam("position/estimated_velocity_threshold",position_parameter.estimated_velocity_threshold);
  n.getParam("position/outlier_threshold",position_parameter.outlier_threshold);
  n.getParam("position/estimated_enu_vel_coefficient",position_parameter.estimated_enu_vel_coefficient);
  n.getParam("position/estimated_position_coefficient",position_parameter.estimated_position_coefficient);
  n.getParam("ecef_base_pos/x",position_parameter.ecef_base_pos_x);
  n.getParam("ecef_base_pos/y",position_parameter.ecef_base_pos_y);
  n.getParam("ecef_base_pos/z",position_parameter.ecef_base_pos_z);

  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "estimated_distance "<<position_parameter.estimated_distance<<std::endl;
  std::cout<< "separation_distance "<<position_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<position_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<position_parameter.outlier_threshold<<std::endl;
  std::cout<< "estimated_enu_vel_coefficient "<<position_parameter.estimated_enu_vel_coefficient<<std::endl;
  std::cout<< "estimated_position_coefficient "<<position_parameter.estimated_position_coefficient<<std::endl;

  ros::Subscriber sub1 = n.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());

  pub = n.advertise<eagleye_msgs::Position>("enu_absolute_pos", 1000);
  ros::spin();

  return 0;
}
