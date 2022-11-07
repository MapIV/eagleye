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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void position_interpolate_estimate(eagleye_msgs::msg::Position enu_absolute_pos, geometry_msgs::msg::Vector3Stamped enu_vel, eagleye_msgs::msg::Position gnss_smooth_pos, eagleye_msgs::msg::Height height,PositionInterpolateParameter position_interpolate_parameter, PositionInterpolateStatus* position_interpolate_status, eagleye_msgs::msg::Position* enu_absolute_pos_interpolate,sensor_msgs::msg::NavSatFix* eagleye_fix)
{

  int i;
  int estimate_index = 0;
  double enu_pos[3],tmp_enu[3];
  double ecef_base_pos[3];
  double ecef_pos[3];
  double llh_pos[3],_llh[3];
  double diff_estimate_enu_pos_x = 0.0;
  double diff_estimate_enu_pos_y = 0.0;
  double diff_estimate_enu_pos_z = 0.0;
  bool position_estimate_status;
  std::size_t imu_stamp_buffer_length;

  double search_buffer_number = position_interpolate_parameter.sync_search_period * position_interpolate_parameter.imu_rate;

  rclcpp::Time ros_clock(enu_absolute_pos.header.stamp);
  rclcpp::Time ros_clock2(enu_vel.header.stamp);
  auto enu_absolute_time = ros_clock.seconds();
  auto enu_vel_time = ros_clock2.seconds();

  enu_absolute_pos_interpolate->ecef_base_pos = enu_absolute_pos.ecef_base_pos;

  if (position_interpolate_status->number_buffer < search_buffer_number)
  {
    ++position_interpolate_status->number_buffer;
  }
  else
  {
    position_interpolate_status->number_buffer = search_buffer_number;
  }

  if (position_interpolate_status->position_stamp_last != enu_absolute_time && enu_absolute_pos.status.estimate_status == true)
  {
    position_estimate_status = true;
    position_interpolate_status->position_estimate_start_status = true;
    ++position_interpolate_status->position_estimate_status_count;
  }
  else
  {
    position_estimate_status = false;
  }

  if(position_interpolate_status->time_last != 0 && std::sqrt((enu_vel.vector.x * enu_vel.vector.x) + (enu_vel.vector.y * enu_vel.vector.y) + (enu_vel.vector.z * enu_vel.vector.z)) > position_interpolate_parameter.stop_judgment_threshold)
  {
    position_interpolate_status->provisional_enu_pos_x = enu_absolute_pos_interpolate->enu_pos.x + enu_vel.vector.x * (enu_vel_time - position_interpolate_status->time_last);
    position_interpolate_status->provisional_enu_pos_y = enu_absolute_pos_interpolate->enu_pos.y + enu_vel.vector.y * (enu_vel_time - position_interpolate_status->time_last);
    position_interpolate_status->provisional_enu_pos_z = enu_absolute_pos_interpolate->enu_pos.z + enu_vel.vector.z * (enu_vel_time - position_interpolate_status->time_last);
  }

  // data buffer generate
  position_interpolate_status->provisional_enu_pos_x_buffer.push_back(position_interpolate_status->provisional_enu_pos_x);
  position_interpolate_status->provisional_enu_pos_y_buffer.push_back(position_interpolate_status->provisional_enu_pos_y);
  position_interpolate_status->provisional_enu_pos_z_buffer.push_back(position_interpolate_status->provisional_enu_pos_z);
  position_interpolate_status->imu_stamp_buffer.push_back(enu_vel_time);
  imu_stamp_buffer_length = std::distance(position_interpolate_status->imu_stamp_buffer.begin(), position_interpolate_status->imu_stamp_buffer.end());

  if (imu_stamp_buffer_length > search_buffer_number)
  {
    position_interpolate_status->provisional_enu_pos_x_buffer.erase(position_interpolate_status->provisional_enu_pos_x_buffer .begin());
    position_interpolate_status->provisional_enu_pos_y_buffer.erase(position_interpolate_status->provisional_enu_pos_y_buffer .begin());
    position_interpolate_status->provisional_enu_pos_z_buffer.erase(position_interpolate_status->provisional_enu_pos_z_buffer .begin());
    position_interpolate_status->imu_stamp_buffer.erase(position_interpolate_status->imu_stamp_buffer .begin());
  }

  if (position_interpolate_status->position_estimate_start_status == true)
  {
    if (position_estimate_status == true)
    {
      for (estimate_index = position_interpolate_status->number_buffer; estimate_index > 0; estimate_index--)
      {
        if (position_interpolate_status->imu_stamp_buffer[estimate_index-1] == enu_absolute_time)
        {
          break;
        }
      }
    }

    if (position_estimate_status == true && estimate_index > 0 && position_interpolate_status->number_buffer >= estimate_index && position_interpolate_status->position_estimate_status_count > 1)
    {
      diff_estimate_enu_pos_x = (position_interpolate_status->provisional_enu_pos_x_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.x);
      diff_estimate_enu_pos_y = (position_interpolate_status->provisional_enu_pos_y_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.y);
      diff_estimate_enu_pos_z = (position_interpolate_status->provisional_enu_pos_z_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.z);
      for (i = estimate_index; i <= position_interpolate_status->number_buffer; i++)
      {
        position_interpolate_status->provisional_enu_pos_x_buffer[i-1] = position_interpolate_status->provisional_enu_pos_x_buffer[i-1] - diff_estimate_enu_pos_x;
        position_interpolate_status->provisional_enu_pos_y_buffer[i-1] = position_interpolate_status->provisional_enu_pos_y_buffer[i-1] - diff_estimate_enu_pos_y;
        position_interpolate_status->provisional_enu_pos_z_buffer[i-1] = position_interpolate_status->provisional_enu_pos_z_buffer[i-1] - diff_estimate_enu_pos_z;
      }
      position_interpolate_status->provisional_enu_pos_x = position_interpolate_status->provisional_enu_pos_x_buffer[position_interpolate_status->number_buffer-1];
      position_interpolate_status->provisional_enu_pos_y = position_interpolate_status->provisional_enu_pos_y_buffer[position_interpolate_status->number_buffer-1];
      position_interpolate_status->provisional_enu_pos_z = position_interpolate_status->provisional_enu_pos_z_buffer[position_interpolate_status->number_buffer-1];

      enu_absolute_pos_interpolate->status.enabled_status = true;
      enu_absolute_pos_interpolate->status.estimate_status = true;
    }
    else if (position_interpolate_status->position_estimate_status_count == 1)
    {
      position_interpolate_status->provisional_enu_pos_x = enu_absolute_pos.enu_pos.x;
      position_interpolate_status->provisional_enu_pos_y = enu_absolute_pos.enu_pos.y;
      position_interpolate_status->provisional_enu_pos_z = enu_absolute_pos.enu_pos.z;
      enu_absolute_pos_interpolate->status.enabled_status = true;
      enu_absolute_pos_interpolate->status.estimate_status = false;
    }
    else
    {
      enu_absolute_pos_interpolate->status.estimate_status = false;
    }
  }

  if (position_interpolate_status->position_estimate_start_status == true)
  {
    enu_pos[0] = position_interpolate_status->provisional_enu_pos_x;
    enu_pos[1] = position_interpolate_status->provisional_enu_pos_y;
    // enu_pos[2] = position_interpolate_status->provisional_enu_pos_z;
    enu_pos[2] = gnss_smooth_pos.enu_pos.z;
    ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;

    enu2llh(enu_pos, ecef_base_pos, llh_pos);

    eagleye_fix->longitude = llh_pos[1] * 180/M_PI;
    eagleye_fix->latitude = llh_pos[0] * 180/M_PI;

    if(height.status.enabled_status == true){
      llh_pos[2] = height.height;

      llh2xyz(llh_pos, ecef_pos);
      xyz2enu(ecef_pos, ecef_base_pos, tmp_enu);

      enu_pos[2] =  tmp_enu[2];
    }

    enu_absolute_pos_interpolate->enu_pos.x = enu_pos[0];
    enu_absolute_pos_interpolate->enu_pos.y = enu_pos[1];
    enu_absolute_pos_interpolate->enu_pos.z = enu_pos[2];

    eagleye_fix->altitude = llh_pos[2];

    // TODO(Map IV): temporary covariance value
    eagleye_fix->position_covariance[0] = 1.5 * 1.5; // [m^2]
    eagleye_fix->position_covariance[4] = 1.5 * 1.5; // [m^2]
    eagleye_fix->position_covariance[8] = 1.5 * 1.5; // [m^2]

  }
  else
  {
    enu_absolute_pos_interpolate->enu_pos.x = 0;
    enu_absolute_pos_interpolate->enu_pos.y = 0;
    enu_absolute_pos_interpolate->enu_pos.z = 0;
    eagleye_fix->longitude = 0;
    eagleye_fix->latitude = 0;
    eagleye_fix->altitude = 0;
    eagleye_fix->position_covariance[0] = 100 * 100; // [m^2]
    eagleye_fix->position_covariance[4] = 100 * 100; // [m^2]
    eagleye_fix->position_covariance[8] = 100 * 100; // [m^2]
  }

  position_interpolate_status->time_last = enu_vel_time;
  position_interpolate_status->position_stamp_last = enu_absolute_time;
}
