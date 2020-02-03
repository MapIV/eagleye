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
#include "eagleye_msgs/Distance.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/Position.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "rtklib_msgs/RtklibNav.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "xyz2enu.hpp"
#include <math.h>
#include <numeric>
#include <time.h>

#include "eagleye_msgs/Debug_log.h"

//default value
static double estimated_distance = 500;
static double separation_distance = 0.1;
static double estimated_velocity_threshold = 10/3.6;
static double outlier_threshold = 5.0;
static double estimated_enu_vel_coefficient = 1.0/10;
static double estimated_position_coefficient = 1.0/50;

static bool data_status, gnss_status;
static int i, count, heading_estimate_status_count;
static int estimated_number = 0;
static int estimated_number_max = estimated_distance/separation_distance;
static int tow_last = 0;
//static int max_index = 0; //pattern1
static int max_x_index, max_y_index; //pattern2,3
static double time_last = 0.0;
static double time_now = 0.0;
static double avg_x, avg_y, avg_z;
static double tmp_enu_pos_x, tmp_enu_pos_y, tmp_enu_pos_z;
static double enu_relative_pos_x, enu_relative_pos_y, enu_relative_pos_z;
static double enu_pos[3];
static double distance_last;

static std::size_t index_length;
static std::size_t velocity_index_length;

static std::size_t debug_velocity_index_length;
static std::size_t distance_index_length;

static std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
static std::vector<double> enu_relative_pos_x_buffer, enu_relative_pos_y_buffer, enu_relative_pos_z_buffer;
static std::vector<double> base_enu_pos_x_buffer, base_enu_pos_y_buffer, base_enu_pos_z_buffer;
static std::vector<double> diff_x_buffer2, diff_y_buffer2, diff_z_buffer2;
static std::vector<double> base_enu_pos_x_buffer2,  base_enu_pos_y_buffer2, base_enu_pos_z_buffer2;
static std::vector<double> diff_x_buffer, diff_y_buffer, diff_z_buffer;
static std::vector<double> diff_buffer;
static std::vector<double> correction_velocity_buffer;
static std::vector<double> distance_buffer;

//static std::vector<double>::iterator max; //pattern1
static std::vector<double>::iterator max_x, max_y; //pattern2,3

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::Distance distance;
static eagleye_msgs::Heading heading_interpolate_3rd;

static eagleye_msgs::Position enu_absolute_pos;
static ros::Publisher pub;

static eagleye_msgs::Debug_log debug;
static ros::Publisher pub_debug;

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

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;

  if(enu_absolute_pos.ecef_base_pos.x == 0 && enu_absolute_pos.ecef_base_pos.y == 0 && enu_absolute_pos.ecef_base_pos.z == 0)
  {
    enu_absolute_pos.ecef_base_pos.x = msg->ecef_pos.x;
    enu_absolute_pos.ecef_base_pos.y = msg->ecef_pos.y;
    enu_absolute_pos.ecef_base_pos.z = msg->ecef_pos.z;
  }

  double ecef_pos[3];
  double ecef_base_pos[3];

  ecef_pos[0] = msg->ecef_pos.x;
  ecef_pos[1] = msg->ecef_pos.y;
  ecef_pos[2] = msg->ecef_pos.z;
  ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
  ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
  ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;

  xyz2enu(ecef_pos, ecef_base_pos, enu_pos);

}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  ++count;

  if (tow_last == rtklib_nav.tow)
  {
    gnss_status = false;
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    tow_last = rtklib_nav.tow;
  }
  else
  {
    gnss_status = true;
    enu_pos[0] = enu_pos[0];
    enu_pos[1] = enu_pos[1];
    enu_pos[2] = enu_pos[2];
    tow_last = rtklib_nav.tow;
  }

  if (heading_interpolate_3rd.status.estimate_status == true)
  {
    heading_interpolate_3rd.status.estimate_status = false; //in order to prevent being judged many times
    ++heading_estimate_status_count;
  }

  if(time_last != 0)
  {
    enu_relative_pos_x = enu_relative_pos_x + msg->vector.x * (msg->header.stamp.toSec() - time_last);
    enu_relative_pos_y = enu_relative_pos_y + msg->vector.y * (msg->header.stamp.toSec() - time_last);
    enu_relative_pos_z = enu_relative_pos_z + msg->vector.z * (msg->header.stamp.toSec() - time_last);
  }


  if (distance.distance-distance_last >= separation_distance && gnss_status == true)
  {

    if (estimated_number < estimated_number_max)
    {
      ++estimated_number;
    }
    else
    {
      estimated_number = estimated_number_max;
    }

    enu_pos_x_buffer.push_back(enu_pos[0]);
    enu_pos_y_buffer.push_back(enu_pos[1]);
    //enu_pos_z_buffer.push_back(enu_pos[2]);
    enu_pos_z_buffer.push_back(0);
    correction_velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    enu_relative_pos_x_buffer.push_back(enu_relative_pos_x);
    enu_relative_pos_y_buffer.push_back(enu_relative_pos_y);
    //enu_relative_pos_z_buffer.push_back(enu_relative_pos_z);
    enu_relative_pos_z_buffer.push_back(0);
    distance_buffer.push_back(distance.distance);

    data_status = true; //judgment that refreshed data

    if (distance_buffer.end() - distance_buffer.begin() > estimated_number_max)
    {
      enu_pos_x_buffer.erase(enu_pos_x_buffer.begin());
      enu_pos_y_buffer.erase(enu_pos_y_buffer.begin());
      enu_pos_z_buffer.erase(enu_pos_z_buffer.begin());
      correction_velocity_buffer.erase(correction_velocity_buffer.begin());
      enu_relative_pos_x_buffer.erase(enu_relative_pos_x_buffer.begin());
      enu_relative_pos_y_buffer.erase(enu_relative_pos_y_buffer.begin());
      enu_relative_pos_z_buffer.erase(enu_relative_pos_z_buffer.begin());
      distance_buffer.erase(distance_buffer.begin());
    }
    distance_last = distance.distance;
  }

 debug.log1 = 0;
 debug.log2 = 0;
 debug.log3 = 0;
 debug.log4 = 0;
 debug.log5 = 0;

  if (data_status == true)
  {
    if (distance.distance > estimated_distance && gnss_status == true && velocity_scale_factor.correction_velocity.linear.x > estimated_velocity_threshold && heading_estimate_status_count > 0 &&
        count > 1)
    {

      std::vector<int> distance_index;
      std::vector<int> velocity_index;
      std::vector<int> index;

      for (i = 0; i < estimated_number; i++)
      {
        if (distance_buffer[estimated_number-1] - distance_buffer[i]  <= estimated_distance)
        {
          distance_index.push_back(i);

          if (correction_velocity_buffer[i] > estimated_velocity_threshold)
          {
            velocity_index.push_back(i);
          }

        }
      }

      set_intersection(velocity_index.begin(), velocity_index.end(), distance_index.begin(), distance_index.end(),
                       inserter(index, index.end()));

      index_length = std::distance(index.begin(), index.end());
      velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

      debug.log1 = index_length;
      debug.log3 = estimated_number;
      debug.log4 = estimated_number;
      debug.log5 = velocity_index_length;

      if (index_length > velocity_index_length * estimated_enu_vel_coefficient)
      {
        while (1)
        {
          index_length = std::distance(index.begin(), index.end());

          base_enu_pos_x_buffer.clear();
          base_enu_pos_y_buffer.clear();
          base_enu_pos_z_buffer.clear();

          for (i = 0; i < estimated_number; i++)
          {
            base_enu_pos_x_buffer.push_back(enu_pos_x_buffer[index[index_length-1]] - enu_relative_pos_x_buffer[index[index_length-1]] +
                                enu_relative_pos_x_buffer[i]);
            base_enu_pos_y_buffer.push_back(enu_pos_y_buffer[index[index_length-1]] - enu_relative_pos_y_buffer[index[index_length-1]] +
                                enu_relative_pos_y_buffer[i]);
            base_enu_pos_z_buffer.push_back(enu_pos_z_buffer[index[index_length-1]] - enu_relative_pos_z_buffer[index[index_length-1]] +
                                enu_relative_pos_z_buffer[i]);
          }

          diff_x_buffer2.clear();
          diff_y_buffer2.clear();
          diff_z_buffer2.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_x_buffer2.push_back(base_enu_pos_x_buffer[index[i]] - enu_pos_x_buffer[index[i]]);
            diff_y_buffer2.push_back(base_enu_pos_y_buffer[index[i]] - enu_pos_y_buffer[index[i]]);
            diff_z_buffer2.push_back(base_enu_pos_z_buffer[index[i]] - enu_pos_z_buffer[index[i]]);
          }

          avg_x = std::accumulate(diff_x_buffer2.begin(), diff_x_buffer2.end(), 0.0) / index_length;
          avg_y = std::accumulate(diff_y_buffer2.begin(), diff_y_buffer2.end(), 0.0) / index_length;
          avg_z = std::accumulate(diff_z_buffer2.begin(), diff_z_buffer2.end(), 0.0) / index_length;

          tmp_enu_pos_x = enu_pos_x_buffer[index[index_length - 1]] - avg_x;
          tmp_enu_pos_y = enu_pos_y_buffer[index[index_length - 1]] - avg_y;
          tmp_enu_pos_z = enu_pos_z_buffer[index[index_length - 1]] - avg_z;

          base_enu_pos_x_buffer2.clear();
          base_enu_pos_y_buffer2.clear();
          base_enu_pos_z_buffer2.clear();

          for (i = 0; i < estimated_number; i++)
          {
            base_enu_pos_x_buffer2.push_back(tmp_enu_pos_x - enu_relative_pos_x_buffer[index[index_length - 1]] + enu_relative_pos_x_buffer[i]);
            base_enu_pos_y_buffer2.push_back(tmp_enu_pos_y - enu_relative_pos_y_buffer[index[index_length - 1]] + enu_relative_pos_y_buffer[i]);
            base_enu_pos_z_buffer2.push_back(tmp_enu_pos_z - enu_relative_pos_z_buffer[index[index_length - 1]] + enu_relative_pos_z_buffer[i]);
          }

          diff_x_buffer.clear();
          diff_y_buffer.clear();
          diff_z_buffer.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_x_buffer.push_back(fabsf(base_enu_pos_x_buffer2[index[i]] - enu_pos_x_buffer[index[i]]));
            diff_y_buffer.push_back(fabsf(base_enu_pos_y_buffer2[index[i]] - enu_pos_y_buffer[index[i]]));
            diff_z_buffer.push_back(fabsf(base_enu_pos_z_buffer2[index[i]] - enu_pos_z_buffer[index[i]]));
          }

          max_x = std::max_element(diff_x_buffer.begin(), diff_x_buffer.end());
          max_y = std::max_element(diff_y_buffer.begin(), diff_y_buffer.end());

          max_x_index = std::distance(diff_x_buffer.begin(), max_x);
          max_y_index = std::distance(diff_y_buffer.begin(), max_y);

          if(diff_x_buffer[max_x_index] < diff_y_buffer[max_y_index])
          {
            if (diff_x_buffer[max_x_index] > outlier_threshold)
            {
              index.erase(index.begin() + max_x_index);
            }
            else
            {
              break;
            }
          }
          else
          {
            if (diff_y_buffer[max_y_index] > outlier_threshold)
            {
              index.erase(index.begin() + max_y_index);
            }
            else
            {
              break;
            }
          }

          index_length = std::distance(index.begin(), index.end());
          velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

          if (index_length < velocity_index_length * estimated_position_coefficient)
          {
            break;
          }

        }

        index_length = std::distance(index.begin(), index.end());
        velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());
        debug.log2 = index_length;

        if (index_length >= velocity_index_length * estimated_position_coefficient)
        {
          if (index[index_length - 1] == estimated_number-1)
          {
            enu_absolute_pos.enu_pos.x = tmp_enu_pos_x;
            enu_absolute_pos.enu_pos.y = tmp_enu_pos_y;
            enu_absolute_pos.enu_pos.z = tmp_enu_pos_z;
            enu_absolute_pos.header = msg->header;
            enu_absolute_pos.status.enabled_status = true;
            enu_absolute_pos.status.estimate_status = true;
            pub.publish(enu_absolute_pos);
          }
          else
          {
            enu_absolute_pos.enu_pos.x = tmp_enu_pos_x + (enu_relative_pos_x_buffer[estimated_number - 1] - enu_relative_pos_x_buffer[index[index_length - 1]]);
            enu_absolute_pos.enu_pos.y = tmp_enu_pos_y + (enu_relative_pos_y_buffer[estimated_number - 1] - enu_relative_pos_y_buffer[index[index_length - 1]]);
            enu_absolute_pos.enu_pos.z = tmp_enu_pos_z + (enu_relative_pos_z_buffer[estimated_number - 1] - enu_relative_pos_z_buffer[index[index_length - 1]]);
          }
        }
      }
    }
  }

  debug.log6 = enu_absolute_pos.status.estimate_status;
  pub_debug.publish(debug);

  time_last = msg->header.stamp.toSec();
  enu_absolute_pos.status.estimate_status = false;
  data_status = false;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position");
  ros::NodeHandle n;

  n.getParam("/eagleye/position/estimated_distance",estimated_distance);
  n.getParam("/eagleye/position/separation_distance",separation_distance);
  n.getParam("/eagleye/position/estimated_velocity_threshold",estimated_velocity_threshold);
  n.getParam("/eagleye/position/outlier_threshold",outlier_threshold);
  n.getParam("/eagleye/position/estimated_enu_vel_coefficient",estimated_enu_vel_coefficient);
  n.getParam("/eagleye/position/estimated_position_coefficient",estimated_position_coefficient);

  std::cout<< "estimated_distance "<<estimated_distance<<std::endl;
  std::cout<< "separation_distance "<<separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<outlier_threshold<<std::endl;
  std::cout<< "estimated_enu_vel_coefficient "<<estimated_enu_vel_coefficient<<std::endl;
  std::cout<< "estimated_position_coefficient "<<estimated_position_coefficient<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/enu_vel", 1000, enu_vel_callback);
  ros::Subscriber sub2 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub4 = n.subscribe("/eagleye/distance", 1000, distance_callback);
  ros::Subscriber sub5 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);

  pub = n.advertise<eagleye_msgs::Position>("/eagleye/enu_absolute_pos", 1000);

  pub_debug = n.advertise<eagleye_msgs::Debug_log>("/eagleye/debug_log", 1000);

  ros::spin();

  return 0;
}
