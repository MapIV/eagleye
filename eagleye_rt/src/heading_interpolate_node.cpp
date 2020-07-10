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
 * heading_interpolate.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "navigation.hpp"

static sensor_msgs::Imu imu;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::YawrateOffset yawrate_offset;
static eagleye_msgs::Heading heading;
static eagleye_msgs::SlipAngle slip_angle;

static ros::Publisher pub;
static eagleye_msgs::Heading heading_interpolate;

struct HeadingInterpolateParameter heading_interpolate_parameter;
struct HeadingInterpolateStatus heading_interpolate_status;

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

void yawrate_offset_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset.header = msg->header;
  yawrate_offset.yawrate_offset = msg->yawrate_offset;
  yawrate_offset.status = msg->status;
}

void heading_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading.header = msg->header;
  heading.heading_angle = msg->heading_angle;
  heading.status = msg->status;
}

void slip_angle_callback(const eagleye_msgs::SlipAngle::ConstPtr& msg)
{
  slip_angle.header = msg->header;
  slip_angle.coefficient = msg->coefficient;
  slip_angle.slip_angle = msg->slip_angle;
  slip_angle.status = msg->status;
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
  heading_interpolate.header = msg->header;
  heading_interpolate_estimate(imu,velocity_scale_factor,yawrate_offset_stop,yawrate_offset,heading,slip_angle,heading_interpolate_parameter,&heading_interpolate_status,&heading_interpolate);
  pub.publish(heading_interpolate);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading_interpolate");
  ros::NodeHandle n("~");

  n.getParam("/eagleye/reverse_imu", heading_interpolate_parameter.reverse_imu);
  n.getParam("/eagleye/heading_interpolate/stop_judgment_velocity_threshold", heading_interpolate_parameter.stop_judgment_velocity_threshold);
  n.getParam("/eagleye/heading_interpolate/number_buffer_max", heading_interpolate_parameter.number_buffer_max);
  std::cout<< "reverse_imu "<<heading_interpolate_parameter.reverse_imu<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<heading_interpolate_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "number_buffer_max "<<heading_interpolate_parameter.number_buffer_max<<std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name_1 = "/subscribe_topic_name/invalid_1";
  std::string subscribe_topic_name_2 = "/subscribe_topic_name/invalid_2";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "/eagleye/heading_interpolate_1st";
      subscribe_topic_name_1 = "/eagleye/yawrate_offset_stop";
      subscribe_topic_name_2 = "/eagleye/heading_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "/eagleye/heading_interpolate_2nd";
      subscribe_topic_name_1 = "/eagleye/yawrate_offset_1st";
      subscribe_topic_name_2 = "/eagleye/heading_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "/eagleye/heading_interpolate_3rd";
      subscribe_topic_name_1 = "/eagleye/yawrate_offset_2nd";
      subscribe_topic_name_2 = "/eagleye/heading_3rd";
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

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_topic_name_1, 1000, yawrate_offset_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_topic_name_2, 1000, heading_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("/eagleye/slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  pub = n.advertise<eagleye_msgs::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
