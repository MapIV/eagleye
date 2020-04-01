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
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include <math.h>

//default value
static bool reverse_imu = false;
static double estimated_number_min = 1500;
static double estimated_number_max = 14000;
static double estimated_coefficient = 0.02;
static double estimated_velocity_threshold = 2.77;

static bool estimate_start_status, estimated_condition_status;
static int i = 0;
static int estimated_preparation_conditions = 0;
static int count = 0;
static int heading_estimate_status_count = 0;
static int estimated_number = 0;
static double time_last = 0.0;
static double sum_xy, sum_x, sum_y, sum_x2;

static double raw_yawrate_offset = 0.0;
static double yawrate_offset_last = 0.0;
static double yawrate = 0.0;

static std::size_t index_length;
static std::size_t time_buffer_length;
static std::size_t inversion_up_index_length;
static std::size_t inversion_down_index_length;
static std::vector<double> time_buffer;
static std::vector<double> yawrate_buffer;
static std::vector<double> heading_angle_buffer;
static std::vector<double> correction_velocity_buffer;
static std::vector<bool> heading_estimate_status_buffer;
static std::vector<double> yawrate_offset_stop_buffer;

static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::Heading heading_interpolate;

static ros::Publisher pub;
static eagleye_msgs::YawrateOffset yawrate_offset;

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
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ++count;
  yawrate_offset.header = msg->header;

  if (estimated_number < estimated_number_max)
  {
    ++estimated_number;
  }
  else
  {
    estimated_number = estimated_number_max;
  }

  if (reverse_imu == false)
  {
    yawrate = msg->angular_velocity.z;
  }
  else if (reverse_imu == true)
  {
    yawrate = -1 * msg->angular_velocity.z;
  }

  // data buffer generate
  time_buffer.push_back(msg->header.stamp.toSec());
  yawrate_buffer.push_back(yawrate);
  heading_angle_buffer.push_back(heading_interpolate.heading_angle);
  correction_velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
  heading_estimate_status_buffer.push_back(heading_interpolate.status.estimate_status);
  yawrate_offset_stop_buffer.push_back(yawrate_offset_stop.yawrate_offset);

  time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());

  if (time_buffer_length > estimated_number_max)
  {
    time_buffer.erase(time_buffer.begin());
    yawrate_buffer.erase(yawrate_buffer.begin());
    heading_angle_buffer.erase(heading_angle_buffer.begin());
    correction_velocity_buffer.erase(correction_velocity_buffer.begin());
    heading_estimate_status_buffer.erase(heading_estimate_status_buffer.begin());
    yawrate_offset_stop_buffer.erase(yawrate_offset_stop_buffer.begin());
  }

  if (estimated_preparation_conditions == 0 && heading_estimate_status_buffer[estimated_number - 1] == true)
  {
    estimated_preparation_conditions = 1;
  }
  else if (estimated_preparation_conditions == 1)
  {
    if (heading_estimate_status_count < estimated_number_min)
    {
      ++heading_estimate_status_count;
    }
    else if (heading_estimate_status_count == estimated_number_min)
    {
      estimated_preparation_conditions = 2;
    }
  }

  if (estimated_preparation_conditions == 2 && correction_velocity_buffer[estimated_number-1] > estimated_velocity_threshold && heading_estimate_status_buffer[estimated_number-1] == true)
  {
    estimated_condition_status = true;
  }
  else
  {
    estimated_condition_status = false;
  }

  std::vector<int> velocity_index;
  std::vector<int> heading_estimate_status_index;
  std::vector<int> index;

  if (estimated_condition_status == true)
  {
    for (i = 0; i < estimated_number; i++)
    {
      if (correction_velocity_buffer[i] > estimated_velocity_threshold)
      {
        velocity_index.push_back(i);
      }
      if (heading_estimate_status_buffer[i] == true)
      {
        heading_estimate_status_index.push_back(i);
      }
    }

    set_intersection(velocity_index.begin(), velocity_index.end(), heading_estimate_status_index.begin(), heading_estimate_status_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > estimated_number * estimated_coefficient)
    {
      std::vector<double> provisional_heading_angle_buffer(estimated_number, 0);

      for (i = 0; i < estimated_number; i++)
      {
        if (i > 0)
        {
          provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] + yawrate_buffer[i] * (time_buffer[i] - time_buffer[i-1]);
        }
      }

      std::vector<double> base_heading_angle_buffer;
      std::vector<double> diff_buffer;
      std::vector<double> time_buffer2;
      std::vector<double> inversion_up_index;
      std::vector<double> inversion_down_index;

      /*
      for (i = 0; i < estimated_number; i++)
      {
        base_heading_angle_buffer.push_back(heading_angle_buffer[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
      }

　　　　
      for (i = 0; i < index_length; i++)
      {
        diff_buffer.push_back(base_heading_angle_buffer[index[i]] - heading_angle_buffer[index[i]]);
      }

      for (i = 0; i < index_length; i++)
      {
        if (diff_buffer[i] > M_PI / 2.0)
        {
          inversion_up_index.push_back(index[i]);
        }
        if (diff_buffer[i] < -M_PI / 2.0)
        {
          inversion_down_index.push_back(index[i]);
        }
      }

      inversion_up_index_length = std::distance(inversion_up_index.begin(), inversion_up_index.end());
      inversion_down_index_length = std::distance(inversion_down_index.begin(), inversion_down_index.end());

      if (inversion_up_index_length != 0)
      {
        for (i = 0; i < inversion_up_index_length; i++)
        {
          heading_angle_buffer[inversion_up_index[i]] = heading_angle_buffer[inversion_up_index[i]] + 2.0 * M_PI;
        }
      }

      if (inversion_down_index_length != 0)
      {
        for (i = 0; i < inversion_down_index_length; i++)
        {
          heading_angle_buffer[inversion_down_index[i]] = heading_angle_buffer[inversion_down_index[i]] - 2.0 * M_PI;
        }
      }
      */

      index_length = std::distance(index.begin(), index.end());

      //base_heading_angle_buffer.clear();
      for (i = 0; i < estimated_number; i++)
      {
        base_heading_angle_buffer.push_back(heading_angle_buffer[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
      }

      //diff_buffer.clear();
      for (i = 0; i < index_length; i++)
      {
        // diff_buffer.push_back(base_heading_angle_buffer[index[i]] - heading_angle_buffer[index[i]]);
        diff_buffer.push_back(heading_angle_buffer[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[index[i]] -
                        heading_angle_buffer[index[i]]);
      }

      time_buffer2.clear();
      for (i = 0; i < index_length; i++)
      {
        time_buffer2.push_back(time_buffer[index[i]] - time_buffer[index[0]]);
      }

      // Least-square
      sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
      for (i = 0; i < index_length ; i++)
      {
        sum_xy += time_buffer2[i] * diff_buffer[i];
        sum_x += time_buffer2[i];
        sum_y += diff_buffer[i];
        sum_x2 += pow(time_buffer2[i], 2);
      }
      yawrate_offset.yawrate_offset = -1 * (index_length * sum_xy - sum_x * sum_y) / (index_length * sum_x2 - pow(sum_x, 2));
      yawrate_offset.status.enabled_status = true;
      yawrate_offset.status.estimate_status = true;
    }
  }

  if (yawrate_offset.status.enabled_status == false)
  {
    yawrate_offset.yawrate_offset = yawrate_offset_stop.yawrate_offset;
  }

  pub.publish(yawrate_offset);
  time_last = msg->header.stamp.toSec();
  yawrate_offset.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yawrate_offset");
  ros::NodeHandle n("~");

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/yawrate_offset/estimated_number_min",estimated_number_min);
  n.getParam("/eagleye/yawrate_offset/estimated_coefficient",estimated_coefficient);
  n.getParam("/eagleye/yawrate_offset/estimated_velocity_threshold",estimated_velocity_threshold);

  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;
  std::cout<< "estimated_number_min "<<estimated_number_min<<std::endl;
  std::cout<< "estimated_coefficient "<<estimated_coefficient<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<estimated_velocity_threshold<<std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "/eagleye/yawrate_offset_1st";
      subscribe_topic_name = "/eagleye/heading_interpolate_1st";
      estimated_number_max = 14000;  // parameters for 1st
      n.getParam("/eagleye/yawrate_offset/1st/estimated_number_max",estimated_number_max);
      std::cout<< "estimated_number_max "<<estimated_number_max<<std::endl;
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "/eagleye/yawrate_offset_2nd";
      subscribe_topic_name = "/eagleye/heading_interpolate_2nd";
      estimated_number_max = 25000;  // parameters for 2nd
      n.getParam("/eagleye/yawrate_offset/2nd/estimated_number_max",estimated_number_max);
      std::cout<< "estimated_number_max "<<estimated_number_max<<std::endl;
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

  ros::Subscriber sub1 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe(subscribe_topic_name, 1000, heading_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("/imu/data_raw", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  pub = n.advertise<eagleye_msgs::YawrateOffset>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
