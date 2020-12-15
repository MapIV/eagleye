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
 * velocity_scale_factor.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static rtklib_msgs::RtklibNav rtklib_nav;
static geometry_msgs::TwistStamped velocity;
static sensor_msgs::Imu imu;


static ros::Publisher pub;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;

struct VelocityScaleFactorParameter velocity_scale_factor_parameter;
struct VelocityScaleFactorStatus velocity_scale_factor_status;


void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  velocity.header = msg->header;
  velocity.twist = msg->twist;
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
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.header.frame_id = "base_link";
  velocity_scale_factor_estimate(rtklib_nav,velocity,velocity_scale_factor_parameter,&velocity_scale_factor_status,&velocity_scale_factor);
  pub.publish(velocity_scale_factor);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_scale_factor");
  ros::NodeHandle n;

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  n.getParam("eagleye/twist_topic",subscribe_twist_topic_name);
  n.getParam("eagleye/imu_topic",subscribe_imu_topic_name);
  n.getParam("eagleye/rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("eagleye/velocity_scale_factor/estimated_number_min",velocity_scale_factor_parameter.estimated_number_min);
  n.getParam("eagleye/velocity_scale_factor/estimated_number_max",velocity_scale_factor_parameter.estimated_number_max);
  n.getParam("eagleye/velocity_scale_factor/estimated_velocity_threshold",velocity_scale_factor_parameter.estimated_velocity_threshold);
  n.getParam("eagleye/velocity_scale_factor/estimated_coefficient",velocity_scale_factor_parameter.estimated_coefficient);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "estimated_number_min "<<velocity_scale_factor_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<velocity_scale_factor_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<velocity_scale_factor_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_coefficient "<<velocity_scale_factor_parameter.estimated_coefficient<<std::endl;

  ros::Subscriber sub1 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  pub = n.advertise<eagleye_msgs::VelocityScaleFactor>("eagleye/velocity_scale_factor", 1000);

  ros::spin();

  return 0;
}
