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
#include "xyz2enu_vel.hpp"

//default value
static bool reverse_imu = false;
static double stop_judgment_velocity_threshold = 0.01;
static bool slip_angle_estimate = true;
static double manual_coefficient = 0;
static double estimated_number_min = 100;
static double estimated_number_max = 1000;
static double estimated_velocity_threshold = 2.77;
static double estimated_yawrate_threshold = 0.017453;

static int i, count, heading_estimate_status_count;
static double time_last;
static double doppler_heading_angle;
static double doppler_slip;
static double yawrate;
static double acceleration_y;
static double estimate_coefficient;
static double sum_xy, sum_x, sum_y, sum_x2;

static std::size_t acceleration_y_buffer_length;
static std::vector<double> acceleration_y_buffer;
static std::vector<double> doppler_slip_buffer;

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::Heading heading_interpolate_3rd;

static ros::Publisher pub;
static eagleye_msgs::SlipAngle slip_angle;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;

  double ecef_vel[3];
  double ecef_pos[3];
  double enu_vel[3];

  ecef_vel[0] = msg->ecef_vel.x;
  ecef_vel[1] = msg->ecef_vel.y;
  ecef_vel[2] = msg->ecef_vel.z;
  ecef_pos[0] = msg->ecef_pos.x;
  ecef_pos[1] = msg->ecef_pos.y;
  ecef_pos[2] = msg->ecef_pos.z;

  xyz2enu_vel(ecef_vel, ecef_pos, enu_vel);
  doppler_heading_angle = atan2(enu_vel[0], enu_vel[1]);
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

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
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
    yawrate = yawrate + yawrate_offset_2nd.yawrate_offset;
  }
  else
  {
    yawrate = yawrate + yawrate_offset_stop.yawrate_offset;
  }

  acceleration_y = velocity_scale_factor.correction_velocity.linear.x * yawrate;

  if (heading_interpolate_3rd.status.estimate_status == true)
  {
    heading_interpolate_3rd.status.estimate_status = false; //in order to prevent being judged many times

    if(heading_estimate_status_count < estimated_number_max)
    {
      ++heading_estimate_status_count;
    }
    else
    {
      heading_estimate_status_count = estimated_number_max;
    }

    if ((velocity_scale_factor.correction_velocity.linear.x > estimated_velocity_threshold) && (fabs(yawrate) > estimated_yawrate_threshold))
    {

      doppler_slip = (fmod(heading_interpolate_3rd.heading_angle,2*M_PI) - fmod(doppler_heading_angle,2*M_PI));

      if (doppler_slip > M_PI / 2.0)
      {
        doppler_slip = doppler_slip + 2.0 * M_PI;
      }
      if (doppler_slip < -M_PI / 2.0)
      {
        doppler_slip = doppler_slip - 2.0 * M_PI;
      }

      if(fabs(doppler_slip)<(2*M_PI/180))
      {
        acceleration_y_buffer.push_back(acceleration_y);
        doppler_slip_buffer.push_back(doppler_slip);

        acceleration_y_buffer_length = std::distance(acceleration_y_buffer.begin(), acceleration_y_buffer.end());

        if (acceleration_y_buffer_length > estimated_number_max)
        {
          acceleration_y_buffer.erase(acceleration_y_buffer.begin());
          doppler_slip_buffer.erase(doppler_slip_buffer.begin());
        }

        if(heading_estimate_status_count > estimated_number_min)
          {
            double sum_xy_avg,sum_x_square = 0.0;
            acceleration_y_buffer_length = std::distance(acceleration_y_buffer.begin(), acceleration_y_buffer.end());

            // Least-square
            sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
            for (i = 0; i < acceleration_y_buffer_length ; i++)
            {
              sum_xy += acceleration_y_buffer[i] * doppler_slip_buffer[i];
              sum_x += acceleration_y_buffer[i];
              sum_y += doppler_slip_buffer[i];
              sum_x2 += pow(acceleration_y_buffer[i], 2);
            }
            estimate_coefficient = (heading_estimate_status_count * sum_xy - sum_x * sum_y) / (heading_estimate_status_count * sum_x2 - pow(sum_x, 2));
            slip_angle.status.enabled_status = true;
            slip_angle.status.estimate_status = true;
          }
        }
      }
    }

  if (velocity_scale_factor.status.enabled_status == true && yawrate_offset_stop.status.enabled_status == true && yawrate_offset_2nd.status.enabled_status == true)
  {
    if (slip_angle_estimate == true)
    {
      slip_angle.coefficient = estimate_coefficient;
      slip_angle.slip_angle = estimate_coefficient * acceleration_y;
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
  slip_angle.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slip_angle");

  ros::NodeHandle n;

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/slip_angle/slip_angle_estimate", slip_angle_estimate);
  n.getParam("/eagleye/slip_angle/manual_coefficient", manual_coefficient);
  n.getParam("/eagleye/slip_angle/estimated_number_min", estimated_number_min);
  n.getParam("/eagleye/slip_angle/estimated_number_max", estimated_number_max);
  n.getParam("/eagleye/slip_angle/estimated_velocity_threshold", estimated_velocity_threshold);
  n.getParam("/eagleye/slip_angle/estimated_yawrate_threshold", estimated_yawrate_threshold);
  n.getParam("/eagleye/slip_angle/stop_judgment_velocity_threshold", stop_judgment_velocity_threshold);
  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;
  std::cout<< "slip_angle_estimate "<<slip_angle_estimate<<std::endl;
  std::cout<< "manual_coefficient "<<manual_coefficient<<std::endl;
  std::cout<< "estimated_number_min "<<estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_yawrate_threshold "<<estimated_yawrate_threshold<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<stop_judgment_velocity_threshold<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback);
  ros::Subscriber sub2 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub4 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  ros::Subscriber sub5 = n.subscribe("/eagleye/yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback);
  ros::Subscriber sub6 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);
  pub = n.advertise<eagleye_msgs::SlipAngle>("/eagleye/slip_angle", 1000);

  ros::spin();

  return 0;
}
