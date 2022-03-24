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
 * yawrate_offset.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::Heading heading_interpolate;
static sensor_msgs::Imu imu;
static ros::Publisher pub;
static eagleye_msgs::YawrateOffset yawrate_offset;

struct YawrateOffsetParameter yawrate_offset_parameter;
struct YawrateOffsetStatus yawrate_offset_status;

bool is_first_heading= false;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void heading_interpolate_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate.header = msg->header;
  heading_interpolate.heading_angle = msg->heading_angle;
  heading_interpolate.status = msg->status;

  if (is_first_heading == false && heading_interpolate.status.enabled_status == true)
  {
    is_first_heading = true;
  }
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (is_first_heading == false)
  {
    return;
  }

  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  yawrate_offset.header = msg->header;
  yawrate_offset_estimate(velocity_scale_factor,yawrate_offset_stop,heading_interpolate,imu, yawrate_offset_parameter, &yawrate_offset_status, &yawrate_offset);
  pub.publish(yawrate_offset);
  yawrate_offset.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yawrate_offset");
  ros::NodeHandle n;

  std::string subscribe_imu_topic_name = "/imu/data_raw";

  n.getParam("imu_topic",subscribe_imu_topic_name);
  n.getParam("reverse_imu", yawrate_offset_parameter.reverse_imu);
  n.getParam("yawrate_offset/estimated_number_min",yawrate_offset_parameter.estimated_number_min);
  n.getParam("yawrate_offset/estimated_coefficient",yawrate_offset_parameter.estimated_coefficient);
  n.getParam("yawrate_offset/estimated_velocity_threshold",yawrate_offset_parameter.estimated_velocity_threshold);
  n.getParam("yawrate_offset/outlier_threshold",yawrate_offset_parameter.outlier_threshold);

  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<yawrate_offset_parameter.reverse_imu<<std::endl;
  std::cout<< "estimated_number_min "<<yawrate_offset_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_coefficient "<<yawrate_offset_parameter.estimated_coefficient<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<yawrate_offset_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<yawrate_offset_parameter.outlier_threshold<<std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "yawrate_offset_1st";
      subscribe_topic_name = "heading_interpolate_1st";
      n.getParam("yawrate_offset/1st/estimated_number_max",yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max "<<yawrate_offset_parameter.estimated_number_max<<std::endl;
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "yawrate_offset_2nd";
      subscribe_topic_name = "heading_interpolate_2nd";
      n.getParam("yawrate_offset/2nd/estimated_number_max",yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max "<<yawrate_offset_parameter.estimated_number_max<<std::endl;
    }
    else
    {
      ROS_ERROR("Invalid argument");
      ros::shutdown();
    }
  }
  else
  {
    ROS_ERROR("No arguments");
    ros::shutdown();
  }

  ros::Subscriber sub1 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe(subscribe_topic_name, 1000, heading_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  pub = n.advertise<eagleye_msgs::YawrateOffset>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
