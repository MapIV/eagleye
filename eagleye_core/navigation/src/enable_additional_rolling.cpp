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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#define g 9.80665

void enable_additional_rolling_estimate(const eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor,
  const eagleye_msgs::msg::YawrateOffset yawrate_offset_2nd,const eagleye_msgs::msg::YawrateOffset yawrate_offset_stop,
  const eagleye_msgs::msg::Distance distance,const sensor_msgs::msg::Imu imu,const geometry_msgs::msg::PoseStamped localization_pose,
  const eagleye_msgs::msg::AngularVelocityOffset angular_velocity_offset_stop,const EnableAdditionalRollingParameter rolling_parameter,
  EnableAdditionalRollingStatus* rolling_status,eagleye_msgs::msg::Rolling* rolling_angle,eagleye_msgs::msg::AccYOffset* acc_y_offset)
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
  double rolling_offset_buffer_num = rolling_parameter.rolling_buffer_num / 2; // Parameter to correct for time delay caused by moving average.

  rclcpp::Time imu_clock(imu.header.stamp);
  double imu_time = imu_clock.seconds();
  rclcpp::Time localization_pose_clock(localization_pose.header.stamp);
  double localization_pose_time = localization_pose_clock.seconds();

  /// reverse_imu ///
  if (!rolling_parameter.reverse_imu)
  {
    rolling_status->yawrate = imu.angular_velocity.z;
  }
  else
  {
    rolling_status->yawrate = -1 * imu.angular_velocity.z;
  }

  /// reverse_imu rollrate ///
  if (!rolling_parameter.reverse_imu_angular_velocity_x)
  {
    rolling_status->rollrate = imu.angular_velocity.x;
    rolling_status->rollrate_offset_stop = angular_velocity_offset_stop.angular_velocity_offset.x;
  }
  else
  {
    rolling_status->rollrate = -1* imu.angular_velocity.x;
    rolling_status->rollrate_offset_stop = -1 * angular_velocity_offset_stop.angular_velocity_offset.x;
  }

  /// reverse_imu y acc ///
  if (!rolling_parameter.reverse_imu_linear_acceleration_y)
  {
    rolling_status->imu_acceleration_y = imu.linear_acceleration.y;
  }
  else
  {
    rolling_status->imu_acceleration_y = -1* imu.linear_acceleration.y;
  }


  // data buffer 
  if (rolling_status->imu_time_buffer.size() < rolling_parameter.imu_buffer_num && velocity_scale_factor.status.enabled_status)
  {
    rolling_status->imu_time_buffer.push_back(imu_time);
    rolling_status->yawrate_buffer.push_back(rolling_status->yawrate);
    rolling_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    rolling_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rolling_status->acceleration_y_buffer.push_back(rolling_status->imu_acceleration_y);
    rolling_status->distance_buffer.push_back(distance.distance);
  }
  else if (velocity_scale_factor.status.enabled_status)
  {
    rolling_status->imu_time_buffer.erase(rolling_status->imu_time_buffer.begin());
    rolling_status->yawrate_buffer.erase(rolling_status->yawrate_buffer.begin());
    rolling_status->velocity_buffer.erase(rolling_status->velocity_buffer.begin());
    rolling_status->yawrate_offset_buffer.erase(rolling_status->yawrate_offset_buffer.begin());
    rolling_status->acceleration_y_buffer.erase(rolling_status->acceleration_y_buffer.begin());
    rolling_status->distance_buffer.erase(rolling_status->distance_buffer.begin());
    
    rolling_status->imu_time_buffer.push_back(imu_time);
    rolling_status->yawrate_buffer.push_back(rolling_status->yawrate);
    rolling_status->velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    rolling_status->yawrate_offset_buffer.push_back(yawrate_offset_2nd.yawrate_offset);
    rolling_status->acceleration_y_buffer.push_back(rolling_status->imu_acceleration_y);
    rolling_status->distance_buffer.push_back(distance.distance);
    data_buffer_status = true;
  }

  /// acc_y_offset ///
  if (data_buffer_status && localization_pose_time != rolling_status->localization_time_last)
  {
    for ( int i = 0; i < rolling_parameter.imu_buffer_num - 1; i++ )
    {
      if (std::abs(rolling_status->imu_time_buffer[i] - localization_pose_time) < rolling_parameter.link_Time_stamp_parameter)
      {
        if (std::abs(rolling_status->distance_last - rolling_status->distance_buffer[i]) >= rolling_parameter.matching_update_distance )
        {
          acc_offset_status = true;
          rolling_status->distance_last = rolling_status->distance_buffer[i];
        }
      }

      if (acc_offset_status)
      {
        if (rolling_status->velocity_buffer[i] > rolling_parameter.stop_judgment_velocity_threshold)
        {
          tf2::Quaternion localization_quat;
          tf2::fromMsg(localization_pose.pose.orientation, localization_quat);
          tf2::Matrix3x3(localization_quat).getRPY(additional_angle[0], additional_angle[1], additional_angle[2]);

          acc_y_offset_tmp = -1*(rolling_status->velocity_buffer[i]*(rolling_status->yawrate_buffer[i]+rolling_status->yawrate_offset_buffer[i])-rolling_status->acceleration_y_buffer[i]-g*std::sin(additional_angle[0]));
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
    if (velocity_scale_factor.correction_velocity.linear.x > rolling_parameter.stop_judgment_velocity_threshold)
    {
      rolling_estimated_tmp = std::asin((velocity_scale_factor.correction_velocity.linear.x*(rolling_status->yawrate+yawrate_offset_2nd.yawrate_offset)/g)-(rolling_status->imu_acceleration_y-acc_y_offset->acc_y_offset)/g);
    }
    else
    {
      rolling_estimated_tmp = std::asin((velocity_scale_factor.correction_velocity.linear.x*(rolling_status->yawrate+yawrate_offset_stop.yawrate_offset)/g)-(rolling_status->imu_acceleration_y-acc_y_offset->acc_y_offset)/g);
    }
  }

  double diff_imu_time = imu_time - rolling_status->imu_time_last;
  double rolling_interpolate_tmp = (rolling_status->rollrate - rolling_status->rollrate_offset_stop)*diff_imu_time;

  /// buffering estimated rolling angle offset ///
  if (rolling_status->roll_rate_interpolate_buffer.size() < rolling_offset_buffer_num)
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
  if (rolling_status->rolling_estimated_buffer.size() < rolling_parameter.rolling_buffer_num && acc_y_offset->status.enabled_status)
  {
    rolling_status->rolling_estimated_buffer.push_back(rolling_estimated_tmp);
  }
  else if (velocity_scale_factor.status.enabled_status && acc_y_offset->status.enabled_status)
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
    for( int i = 0; i <rolling_offset_buffer_num - 1; i++)
    {
      rolling_interpolate += rolling_status->roll_rate_interpolate_buffer[i];
    }
  }

  /// Moving average estimation of roll angle ///
  if (rolling_estimated_buffer_status )
  {
    for ( int i = 0; i <rolling_parameter.rolling_buffer_num - 1; i++ )
    {
      rolling_estimated_sum =rolling_estimated_sum + rolling_status->rolling_estimated_buffer[i];
    }
    rolling_estimated_average = rolling_estimated_sum/rolling_parameter.rolling_buffer_num;
    rolling_angle->rolling_angle = rolling_estimated_average + rolling_interpolate;
    rolling_angle->status.enabled_status=true;
    rolling_angle->status.estimate_status=true;
  }
  else
  {
    rolling_angle->status.estimate_status=false;
  }

  rolling_status->imu_time_last = imu_time;
  rolling_status->localization_time_last = localization_pose_time;
}
