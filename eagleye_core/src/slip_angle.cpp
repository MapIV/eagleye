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
 * slip_angle.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "eagleye_msgs/SlipAngle.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Heading.h"
#include "rtklib_msgs/RtklibNav.h"
#include "sensor_msgs/Imu.h"

//default value
static bool reverse_imu = false;
static double stop_judgment_velocity_threshold = 0.01;
static bool slip_angle_estimate = true;
static double manual_coefficient = 0;

static int count;
static double time_last;
static double yawrate;
static double acceleration_y;

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::YawrateOffset yawrate_offset;
static eagleye_msgs::Heading heading_interpolate;

static ros::Publisher pub;
static eagleye_msgs::SlipAngle slip_angle;

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

void heading_interpolate_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate.header = msg->header;
  heading_interpolate.heading_angle = msg->heading_angle;
  heading_interpolate.status = msg->status;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  slip_angle.header = msg->header;

  ++count;

  if (reverse_imu == false)
  {
    yawrate = msg->angular_velocity.z;
  }
  else if (reverse_imu == true)
  {
    yawrate = -1 * msg->angular_velocity.z;
  }

  if (velocity_scale_factor.correction_velocity.linear.x > stop_judgment_velocity_threshold)
  {
    yawrate = yawrate + yawrate_offset.yawrate_offset;
  }
  else
  {
    yawrate = yawrate + yawrate_offset_stop.yawrate_offset;
  }

  if (velocity_scale_factor.status.enabled_status == true && yawrate_offset_stop.status.enabled_status == true && yawrate_offset.status.enabled_status == true)
  {

    acceleration_y = velocity_scale_factor.correction_velocity.linear.x * yawrate;

    if (slip_angle_estimate == true)
    {
      ROS_ERROR("Unimplemented!!!");
    }
    else
    {
      slip_angle.coefficient = manual_coefficient;
      slip_angle.slip_angle = manual_coefficient * acceleration_y;
      slip_angle.status.enabled_status = false;
      slip_angle.status.estimate_status = false;
    }
  }
  pub.publish(slip_angle);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slip_angle");

  ros::NodeHandle n;

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/slip_angle/stop_judgment_velocity_threshold", stop_judgment_velocity_threshold);
  n.getParam("/eagleye/slip_angle/slip_angle_estimate", slip_angle_estimate);
  n.getParam("/eagleye/slip_angle/manual_coefficient", manual_coefficient);
  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "slip_angle_estimate "<<slip_angle_estimate<<std::endl;
  std::cout<< "manual_coefficient "<<manual_coefficient<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback);
  ros::Subscriber sub2 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub4 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  ros::Subscriber sub5 = n.subscribe("/eagleye/yawrate_offset_2nd", 1000, yawrate_offset_callback);
  ros::Subscriber sub6 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, heading_interpolate_callback);
  pub = n.advertise<eagleye_msgs::SlipAngle>("/eagleye/slip_angle", 1000);

  ros::spin();

  return 0;
}
