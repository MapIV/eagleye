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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "navigation.hpp"

static sensor_msgs::Imu imu;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::Heading heading_interpolate_3rd;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::Pitching pitching;


static geometry_msgs::Vector3Stamped enu_vel;
static eagleye_msgs::Position enu_relative_pos;
static geometry_msgs::TwistStamped eagleye_twist;
static ros::Publisher pub1;
static ros::Publisher pub2;
static ros::Publisher pub3;

struct TrajectoryParameter trajectory_parameter;
struct TrajectoryStatus trajectory_status;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void pitching_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  pitching.header = msg->header;
  pitching.pitching_angle = msg->heading_angle;
  pitching.status = msg->status;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  enu_vel.header = msg->header;
  enu_relative_pos.header = msg->header;
  eagleye_twist.header = msg->header;
  trajectory3d_estimate(imu,velocity_scale_factor,heading_interpolate_3rd,yawrate_offset_stop,yawrate_offset_2nd,pitching,trajectory_parameter,&trajectory_status,&enu_vel,&enu_relative_pos,&eagleye_twist);
  pub1.publish(enu_vel);
  pub2.publish(enu_relative_pos);
  pub3.publish(eagleye_twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");

  ros::NodeHandle n;

  n.getParam("/eagleye/reverse_imu", trajectory_parameter.reverse_imu);
  n.getParam("/eagleye/trajectory/stop_judgment_velocity_threshold",trajectory_parameter.stop_judgment_velocity_threshold);
  std::cout<< "reverse_imu "<<trajectory_parameter.reverse_imu<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe("/eagleye/yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("/eagleye/pitching", 1000, pitching_callback, ros::TransportHints().tcpNoDelay());
  pub1 = n.advertise<geometry_msgs::Vector3Stamped>("/eagleye/enu_vel", 1000);
  pub2 = n.advertise<eagleye_msgs::Position>("/eagleye/enu_relative_pos", 1000);
  pub3 = n.advertise<geometry_msgs::TwistStamped>("/eagleye/twist", 1000);

  ros::spin();

  return 0;
}
