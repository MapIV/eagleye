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
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "eagleye_msgs/SlipAngle.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include <boost/circular_buffer.hpp>
#include <math.h>
#include <numeric>

//default value
static bool reverse_imu = false;
static double stop_judgment_velocity_threshold = 0.01;
static double number_buffer_max = 100;

static bool heading_estimate_status, heading_estimate_start_status;
static int i, count, heading_estimate_status_count;
static int estimate_index = 0;
static int number_buffer = 0;

static double heading_stamp_last = 0;
static double time_last = 0.0;
static double yawrate = 0.0;
static double provisional_heading_angle = 0.0;
static double diff_estimate_heading_angle = 0.0;

static boost::circular_buffer<double> provisional_heading_angle_buffer(number_buffer_max);
static boost::circular_buffer<double> imu_stamp_buffer(number_buffer_max);

static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::YawrateOffset yawrate_offset;
static eagleye_msgs::Heading heading;
static eagleye_msgs::SlipAngle slip_angle;

static ros::Publisher pub;
static eagleye_msgs::Heading heading_interpolate;

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
  heading_interpolate.header = msg->header;

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

  if (number_buffer < number_buffer_max)
  {
    ++number_buffer;
  }
  else
  {
    number_buffer = number_buffer_max;
  }

  if (heading_stamp_last == heading.header.stamp.toSec())
  {
    heading_estimate_status = false;
  }
  else
  {
    heading_estimate_status = true;
    heading_estimate_start_status = true;
    ++heading_estimate_status_count;
  }

  if(time_last != 0)
  {
    provisional_heading_angle = provisional_heading_angle + (yawrate * (msg->header.stamp.toSec() - time_last));
  }

  provisional_heading_angle_buffer.push_back(provisional_heading_angle);
  imu_stamp_buffer.push_back(msg->header.stamp.toSec());

  if (heading_estimate_start_status == true)
  {
    if (heading_estimate_status == true)
    {
      for (estimate_index = number_buffer; estimate_index > 0; estimate_index--)
      {
        if (imu_stamp_buffer[estimate_index-1] == heading.header.stamp.toSec())
        {
          break;
        }
      }
    }

    if (heading_estimate_status == true && estimate_index > 0 && number_buffer >= estimate_index && heading_estimate_status_count > 1)
    {
      diff_estimate_heading_angle = (provisional_heading_angle_buffer[estimate_index-1] - heading.heading_angle);
      for (i = estimate_index; i <= number_buffer; i++)
      {
        provisional_heading_angle_buffer[i-1] = provisional_heading_angle_buffer[i-1] - diff_estimate_heading_angle;
      }
      provisional_heading_angle = provisional_heading_angle_buffer[number_buffer-1];

      heading_interpolate.status.enabled_status = true;
      heading_interpolate.status.estimate_status = true;
    }
    else if (heading_estimate_status_count == 1)
    {
      provisional_heading_angle = heading.heading_angle;
      heading_interpolate.status.enabled_status = true;
      heading_interpolate.status.estimate_status = true;
    }
    else if (count > 1)
    {
      heading_interpolate.status.estimate_status = false;
    }
  }

  /*
  // angle reversal processing (-3.14~3.14)
  if (provisional_heading_angle > M_PI)
  {
    provisional_heading_angle = provisional_heading_angle - 2.0 * M_PI;
  }
  else if (provisional_heading_angle < -M_PI)
  {
    provisional_heading_angle = provisional_heading_angle + 2.0 * M_PI;
  }
  */

  if (heading_estimate_start_status == true)
  {
    heading_interpolate.heading_angle = provisional_heading_angle + slip_angle.slip_angle;
  }
  else
  {
    heading_interpolate.heading_angle = 0.0;
    heading_interpolate.status.enabled_status = false;
    heading_interpolate.status.estimate_status = false;
  }

  pub.publish(heading_interpolate);

  time_last = msg->header.stamp.toSec();
  heading_stamp_last = heading.header.stamp.toSec();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading_interpolate");
  ros::NodeHandle n("~");

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/heading_interpolate/stop_judgment_velocity_threshold", stop_judgment_velocity_threshold);
  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<stop_judgment_velocity_threshold<<std::endl;

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

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  ros::Subscriber sub4 = n.subscribe(subscribe_topic_name_1, 1000, yawrate_offset_callback);
  ros::Subscriber sub5 = n.subscribe(subscribe_topic_name_2, 1000, heading_callback);
  ros::Subscriber sub6 = n.subscribe("/eagleye/slip_angle", 1000, slip_angle_callback);
  pub = n.advertise<eagleye_msgs::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
