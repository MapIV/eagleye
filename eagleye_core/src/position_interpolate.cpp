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
 * position_interpolate.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "eagleye_msgs/Position.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "enu2llh.hpp"
#include "hgeoid.hpp"
#include <boost/circular_buffer.hpp>

static double number_buffer_max = 100;
static bool position_estimate_status, position_estimate_start_status;
static int i, count, position_estimate_status_count;
static int estimate_index = 0;
static int number_buffer = 0;
static bool altitude_estimate = true;


static double position_stamp_last = 0;
static double time_last = 0.0;
static double provisional_enu_pos_x = 0.0;
static double provisional_enu_pos_y = 0.0;
static double provisional_enu_pos_z = 0.0;
static double diff_estimate_enu_pos_x = 0.0;
static double diff_estimate_enu_pos_y = 0.0;
static double diff_estimate_enu_pos_z = 0.0;

static boost::circular_buffer<double> provisional_enu_pos_x_buffer(number_buffer_max);
static boost::circular_buffer<double> provisional_enu_pos_y_buffer(number_buffer_max);
static boost::circular_buffer<double> provisional_enu_pos_z_buffer(number_buffer_max);
static boost::circular_buffer<double> imu_stamp_buffer(number_buffer_max);

static eagleye_msgs::Position enu_absolute_pos;

static eagleye_msgs::Position enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix eagleye_fix;
static ros::Publisher pub1;
static ros::Publisher pub2;

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_absolute_pos_interpolate.header = msg->header;
  enu_absolute_pos_interpolate.ecef_base_pos = enu_absolute_pos.ecef_base_pos;
  eagleye_fix.header = msg->header;

  ++count;

  if (number_buffer < number_buffer_max)
  {
    ++number_buffer;
  }
  else
  {
    number_buffer = number_buffer_max;
  }

  if (position_stamp_last == enu_absolute_pos.header.stamp.toSec())
  {
    position_estimate_status = false;
  }
  else
  {
    position_estimate_status = true;
    position_estimate_start_status = true;
    ++position_estimate_status_count;
  }

  if(time_last != 0)
  {
    provisional_enu_pos_x = enu_absolute_pos_interpolate.enu_pos.x + msg->vector.x * (msg->header.stamp.toSec() - time_last);
    provisional_enu_pos_y = enu_absolute_pos_interpolate.enu_pos.y + msg->vector.y * (msg->header.stamp.toSec() - time_last);
    provisional_enu_pos_z = enu_absolute_pos_interpolate.enu_pos.z + msg->vector.z * (msg->header.stamp.toSec() - time_last);
  }

  provisional_enu_pos_x_buffer.push_back(provisional_enu_pos_x);
  provisional_enu_pos_y_buffer.push_back(provisional_enu_pos_y);
  provisional_enu_pos_z_buffer.push_back(provisional_enu_pos_z);
  imu_stamp_buffer.push_back(msg->header.stamp.toSec());

  if (position_estimate_start_status == true)
  {
    if (position_estimate_status == true)
    {
      for (estimate_index = number_buffer; estimate_index > 0; estimate_index--)
      {
        if (imu_stamp_buffer[estimate_index-1] == enu_absolute_pos.header.stamp.toSec())
        {
          break;
        }
      }
    }

    if (position_estimate_status == true && estimate_index > 0 && number_buffer >= estimate_index && position_estimate_status_count > 1)
    {
      diff_estimate_enu_pos_x = (provisional_enu_pos_x_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.x);
      diff_estimate_enu_pos_y = (provisional_enu_pos_y_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.y);
      diff_estimate_enu_pos_z = (provisional_enu_pos_z_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.z);
      for (i = estimate_index; i <= number_buffer; i++)
      {
        provisional_enu_pos_x_buffer[i-1] = provisional_enu_pos_x_buffer[i-1] - diff_estimate_enu_pos_x;
        provisional_enu_pos_y_buffer[i-1] = provisional_enu_pos_y_buffer[i-1] - diff_estimate_enu_pos_y;
        provisional_enu_pos_z_buffer[i-1] = provisional_enu_pos_z_buffer[i-1] - diff_estimate_enu_pos_z;
      }
      provisional_enu_pos_x = provisional_enu_pos_x_buffer[number_buffer-1];
      provisional_enu_pos_y = provisional_enu_pos_y_buffer[number_buffer-1];
      provisional_enu_pos_z = provisional_enu_pos_z_buffer[number_buffer-1];

      enu_absolute_pos_interpolate.status.enabled_status = true;
      enu_absolute_pos_interpolate.status.estimate_status = true;
    }
    else if (position_estimate_status_count == 1)
    {
      provisional_enu_pos_x = enu_absolute_pos.enu_pos.x;
      provisional_enu_pos_y = enu_absolute_pos.enu_pos.y;
      provisional_enu_pos_z = enu_absolute_pos.enu_pos.z;
      enu_absolute_pos_interpolate.status.enabled_status = true;
      enu_absolute_pos_interpolate.status.estimate_status = false;
    }
    else if (count > 1)
    {
      enu_absolute_pos_interpolate.status.estimate_status = false;
    }
  }

  if (position_estimate_status_count > 1)
  {
    enu_absolute_pos_interpolate.enu_pos.x = provisional_enu_pos_x;
    enu_absolute_pos_interpolate.enu_pos.y = provisional_enu_pos_y;
    enu_absolute_pos_interpolate.enu_pos.z = provisional_enu_pos_z;

    double enu_pos[3];
    double ecef_base_pos[3];
    double llh_pos[3],_llh[3];
    double height;

    enu_pos[0] = enu_absolute_pos_interpolate.enu_pos.x;
    enu_pos[1] = enu_absolute_pos_interpolate.enu_pos.y;
    enu_pos[2] = enu_absolute_pos_interpolate.enu_pos.z;
    ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;

    //enu_pos[2] = 0;


    enu2llh(enu_pos, ecef_base_pos, llh_pos);

    if (altitude_estimate = true)
    {
      _llh[0] = llh_pos[1];
      _llh[1] = llh_pos[0];
      _llh[2] = llh_pos[2];
      hgeoid(_llh,&height);
      llh_pos[2] = llh_pos[2] - height;
    }

    eagleye_fix.longitude = llh_pos[0];
    eagleye_fix.latitude = llh_pos[1];
    //eagleye_fix.altitude = 0;
    eagleye_fix.altitude = llh_pos[2];

    pub2.publish(eagleye_fix);
  }
  pub1.publish(enu_absolute_pos_interpolate);
  time_last = msg->header.stamp.toSec();
  position_stamp_last = enu_absolute_pos.header.stamp.toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_interpolate");
  ros::NodeHandle n;

  n.getParam("/eagleye/position/altitude_estimate",altitude_estimate);
  std::cout<< "altitude_estimate "<<altitude_estimate<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/enu_vel", 1000, enu_vel_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback);
  pub1 = n.advertise<eagleye_msgs::Position>("/eagleye/enu_absolute_pos_interpolate", 1000);
  pub2 = n.advertise<sensor_msgs::NavSatFix>("/eagleye/fix", 1000);

  ros::spin();

  return 0;
}
