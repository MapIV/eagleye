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
 * Ver 1.00 2019/1/24
 */

#include "ros/ros.h"
#include <boost/circular_buffer.hpp>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "eagleye_msgs/YawrateOffset.h"

//default value
static bool reverse_imu = false;
static double stop_judgment_velocity_threshold = 0.01;
static double estimated_number = 200;

static int i, stop_count;
static double tmp = 0.0;
static double yawrate_offset_stop_last = 0.0;

static std::size_t yawrate_buffer_length;
static std::vector<double> yawrate_buffer;

static geometry_msgs::TwistStamped velocity;

static ros::Publisher pub;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  velocity.header = msg->header;
  velocity.twist = msg->twist;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  yawrate_offset_stop.header = msg->header;

  // data buffer generate
  if (reverse_imu == false)
  {
    yawrate_buffer.push_back(msg->angular_velocity.z);
  }
  else if (reverse_imu == true)
  {
    yawrate_buffer.push_back(-1 * msg->angular_velocity.z);
  }

  yawrate_buffer_length = std::distance(yawrate_buffer.begin(), yawrate_buffer.end());

  if (yawrate_buffer_length > estimated_number)
  {
    yawrate_buffer.erase(yawrate_buffer.begin());
  }

  if (velocity.twist.linear.x < stop_judgment_velocity_threshold)
  {
    ++stop_count;
  }
  else
  {
    stop_count = 0;
  }

  // mean
  if (stop_count > estimated_number)
  {
    tmp = 0.0;
    for (i = 0; i < estimated_number; i++)
    {
      tmp += yawrate_buffer[i];
    }
    yawrate_offset_stop.yawrate_offset = -1 * tmp / estimated_number;
    yawrate_offset_stop.status.enabled_status = true;
    yawrate_offset_stop.status.estimate_status = true;
  }
  else
  {
    yawrate_offset_stop.yawrate_offset = yawrate_offset_stop_last;
    yawrate_offset_stop.status.estimate_status = false;
  }

  pub.publish(yawrate_offset_stop);
  yawrate_offset_stop_last = yawrate_offset_stop.yawrate_offset;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yawrate_offset_stop");
  ros::NodeHandle n("~");

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/YawrateOffsetStop/stop_judgment_velocity_threshold",stop_judgment_velocity_threshold);
  n.getParam("/eagleye/YawrateOffsetStop/estimated_number",estimated_number);

  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "estimated_number "<<estimated_number<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/can_twist", 1000, velocity_callback);
  ros::Subscriber sub2 = n.subscribe("/imu/data_raw", 1000, imu_callback);
  pub = n.advertise<eagleye_msgs::YawrateOffset>("/eagleye/yawrate_offset_stop", 1000);

  ros::spin();

  return 0;
}
