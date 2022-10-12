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
 * enable_additional_rolling.cpp
 * Author MapIV Hoda
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#define g 9.80665

void enable_additional_rolling_estimate(const geometry_msgs::TwistStamped velocity,const eagleye_msgs::StatusStamped velocity_status,
  const eagleye_msgs::YawrateOffset yawrate_offset_2nd,const eagleye_msgs::YawrateOffset yawrate_offset_stop,const eagleye_msgs::Distance distance,
  const sensor_msgs::Imu imu, const geometry_msgs::PoseStamped localization_pose,const eagleye_msgs::AngularVelocityOffset angular_velocity_offset_stop,
  const EnableAdditionalRollingParameter rolling_parameter,EnableAdditionalRollingStatus* rolling_status,
  eagleye_msgs::Rolling* rolling_angle,eagleye_msgs::AccYOffset* acc_y_offset)
{
  bool acc_offset_status = false;
  bool rolling_buffer_status = false;
  bool rolling_estimated_buffer_status = false;
  bool data_buffer_status = false;
  double additional_angle[3];
  double acc_y_offset_tmp = 0.0;
  double rolling_estimated_tmp = 0.0;
  double rolling_estimated_sum = 0.0;
  double rolling_estimated_average = 0.0;
  double rolling_interpolate = 0.0;

  double moving_average_buffer_number = rolling_parameter.moving_average_time * rolling_parameter.imu_rate;
  double search_buffer_number = rolling_parameter.sync_search_period * rolling_parameter.imu_rate;
  double rolling_delay_interpolation_buffer_num = search_buffer_number / 2; // Parameter to correct for time delay caused by moving average.

  rolling_status->yawrate = imu.angular_velocity.z;
  rolling_status->rollrate = imu.angular_velocity.x;
  rolling_status->rollrate_offset_stop = angular_velocity_offset_stop.angular_velocity_offset.x;

  rolling_status->imu_acceleration_y = imu.linear_acceleration.y;

  // data buffer 
  if (rolling_status->imu_time_buffer.size() < search_buffer_number && velocity_status.status.enabled_status)
  {
    rolling_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    rolling_status->yawrate_buffer.push_back(rolling_status->yawrate);
    rolling_status->velocity_buffer.push_back(velocity.twist.linear.x);
    rolling_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rolling_status->acceleration_y_buffer.push_back(rolling_status->imu_acceleration_y);
    rolling_status->distance_buffer.push_back(distance.distance);
  }
  else if (velocity_status.status.enabled_status)
  {
    rolling_status->imu_time_buffer.erase(rolling_status->imu_time_buffer.begin());
    rolling_status->yawrate_buffer.erase(rolling_status->yawrate_buffer.begin());
    rolling_status->velocity_buffer.erase(rolling_status->velocity_buffer.begin());
    rolling_status->yawrate_offset_buffer.erase(rolling_status->yawrate_offset_buffer.begin());
    rolling_status->acceleration_y_buffer.erase(rolling_status->acceleration_y_buffer.begin());
    rolling_status->distance_buffer.erase(rolling_status->distance_buffer.begin());
    
    rolling_status->imu_time_buffer.push_back(imu.header.stamp.toSec());
    rolling_status->yawrate_buffer.push_back(rolling_status->yawrate);
    rolling_status->velocity_buffer.push_back(velocity.twist.linear.x);
    rolling_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rolling_status->acceleration_y_buffer.push_back(rolling_status->imu_acceleration_y);
    rolling_status->distance_buffer.push_back(distance.distance);
    data_buffer_status = true;
  }

  /// acc_y_offset ///
  if (data_buffer_status && localization_pose.header.stamp.toSec() != rolling_status->localization_time_last)
  {
    for ( int i = 0; i < search_buffer_number - 1; i++ )
    {
      if (std::abs(rolling_status->imu_time_buffer[i] - localization_pose.header.stamp.toSec()) < rolling_parameter.sync_judgment_threshold)
      {
        if (std::abs(rolling_status->distance_last - rolling_status->distance_buffer[i]) >= rolling_parameter.update_distance )
        {
          acc_offset_status = true;
          rolling_status->distance_last = rolling_status->distance_buffer[i];
        }
      }

      if (acc_offset_status)
      {
        if (rolling_status->velocity_buffer[i] > rolling_parameter.stop_judgment_threshold)
        {
          tf::Quaternion localization_quat;
          quaternionMsgToTF(localization_pose.pose.orientation, localization_quat);
          tf::Matrix3x3(localization_quat).getRPY(additional_angle[0], additional_angle[1], additional_angle[2]);

          acc_y_offset_tmp = -1*(rolling_status->velocity_buffer[i]*(rolling_status->yawrate_buffer[i]+rolling_status->yawrate_offset_buffer[i])-
            rolling_status->acceleration_y_buffer[i]-g*std::sin(additional_angle[0]));
          rolling_status->acc_offset_sum = rolling_status->acc_offset_sum + acc_y_offset_tmp;
          rolling_status->acc_offset_data_count ++;
          acc_y_offset->acc_y_offset = rolling_status->acc_offset_sum/rolling_status->acc_offset_data_count;
          acc_offset_status = false;
          acc_y_offset->status.enabled_status=true;
          acc_y_offset->status.estimate_status=true;
          break;
        }
      }
      else
      {
        acc_y_offset->status.estimate_status=false;
      }

    }
  }else
  {
    acc_y_offset->status.estimate_status=false;
  }

  /// estimated rolling angle ///
  if (acc_y_offset->status.enabled_status)
  {
    if (velocity.twist.linear.x > rolling_parameter.stop_judgment_threshold)
    {
      rolling_estimated_tmp = std::asin((velocity.twist.linear.x*(rolling_status->yawrate+yawrate_offset_2nd.yawrate_offset)/g)-
        (rolling_status->imu_acceleration_y-acc_y_offset->acc_y_offset)/g);
    }
    else
    {
      rolling_estimated_tmp = std::asin((velocity.twist.linear.x*(rolling_status->yawrate+yawrate_offset_stop.yawrate_offset)/g)-
        (rolling_status->imu_acceleration_y-acc_y_offset->acc_y_offset)/g);
    }
  }

  double diff_imu_time = imu.header.stamp.toSec() - rolling_status->imu_time_last;
  double rolling_interpolate_tmp = (rolling_status->rollrate - rolling_status->rollrate_offset_stop)*diff_imu_time;

  /// buffering estimated rolling angle offset ///
  if (rolling_status->roll_rate_interpolate_buffer.size() < rolling_delay_interpolation_buffer_num)
  {
    rolling_status->roll_rate_interpolate_buffer.push_back(rolling_interpolate_tmp);
  }
  else 
  {
    rolling_status->roll_rate_interpolate_buffer.erase(rolling_status->roll_rate_interpolate_buffer.begin());
    rolling_status->roll_rate_interpolate_buffer.push_back(rolling_interpolate_tmp);
    rolling_buffer_status = true;
  }


  /// buffering estimated roll angle ///
  if (rolling_status->rolling_estimated_buffer.size() < moving_average_buffer_number && acc_y_offset->status.enabled_status)
  {
    rolling_status->rolling_estimated_buffer.push_back(rolling_estimated_tmp);
  }
  else if (velocity_status.status.enabled_status && acc_y_offset->status.enabled_status)
  {
    rolling_status->rolling_estimated_buffer.erase(rolling_status->rolling_estimated_buffer.begin());
    rolling_status->rolling_estimated_buffer.push_back(rolling_estimated_tmp);
    rolling_estimated_buffer_status = true;
  }
  else
  {
    rolling_estimated_buffer_status = false;
  }

  /// buffering rolling offset ///
  if (rolling_buffer_status)
  {
    for( int i = 0; i <rolling_delay_interpolation_buffer_num - 1; i++)
    {
      rolling_interpolate += rolling_status->roll_rate_interpolate_buffer[i];
    }
  }

  /// Moving average estimation of roll angle ///
  if (rolling_estimated_buffer_status )
  {
    for ( int i = 0; i <moving_average_buffer_number - 1; i++ )
    {
      rolling_estimated_sum =rolling_estimated_sum + rolling_status->rolling_estimated_buffer[i];
    }
    rolling_estimated_average = rolling_estimated_sum/moving_average_buffer_number;
    rolling_angle->rolling_angle = rolling_estimated_average + rolling_interpolate;
    rolling_angle->status.enabled_status=true;
    rolling_angle->status.estimate_status=true;
  }
  else
  {
    rolling_angle->status.estimate_status=false;
  }

  rolling_status->imu_time_last = imu.header.stamp.toSec();
  rolling_status->localization_time_last = localization_pose.header.stamp.toSec();
}