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
 * angular_velocity_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "navigation.hpp"

void angular_velocity_offset_stop_estimate(const geometry_msgs::TwistStamped velocity, const sensor_msgs::Imu imu, const AngularVelocityOffsetStopParameter angular_velocity_stop_parameter, AngularVelocityOffsetStopStatus* angular_velocity_stop_status, eagleye_msgs::AngularVelocityOffset* angular_velocity_offset_stop)
{

  int i;
  double roll_tmp, pitch_tmp, yaw_tmp;
  double initial_angular_velocity_offset_stop = 0.0;
  double estimated_time_buffer_num = angular_velocity_stop_parameter.estimated_number;
  std::size_t rollrate_buffer_length;
  std::size_t pitchrate_buffer_length;
  std::size_t yawrate_buffer_length;

  // data buffer generate
  if (angular_velocity_stop_status->estimate_start_status == false)
  {
    if (angular_velocity_stop_parameter.reverse_imu == false)
    {
      angular_velocity_stop_status->rollrate_buffer.push_back(imu.angular_velocity.x);
      angular_velocity_stop_status->pitchrate_buffer.push_back(imu.angular_velocity.y);
      angular_velocity_stop_status->yawrate_buffer.push_back(imu.angular_velocity.z);
    }
    else if (angular_velocity_stop_parameter.reverse_imu == true)
    {
      angular_velocity_stop_status->rollrate_buffer.push_back(imu.angular_velocity.x);
      angular_velocity_stop_status->pitchrate_buffer.push_back(imu.angular_velocity.y);
      angular_velocity_stop_status->yawrate_buffer.push_back(-1 * imu.angular_velocity.z);
    }
  }
  else if ( fabs(fabs(angular_velocity_stop_status->yawrate_offset_stop_last) - fabs(imu.angular_velocity.z)) < angular_velocity_stop_parameter.outlier_threshold && angular_velocity_stop_status->estimate_start_status == true)
  {
    if (angular_velocity_stop_parameter.reverse_imu == false)
    {
      angular_velocity_stop_status->rollrate_buffer.push_back(imu.angular_velocity.x);
      angular_velocity_stop_status->pitchrate_buffer.push_back(imu.angular_velocity.y);
      angular_velocity_stop_status->yawrate_buffer.push_back(imu.angular_velocity.z);
    }
    else if (angular_velocity_stop_parameter.reverse_imu == true)
    {
      angular_velocity_stop_status->rollrate_buffer.push_back(imu.angular_velocity.x);
      angular_velocity_stop_status->pitchrate_buffer.push_back(imu.angular_velocity.y);
      angular_velocity_stop_status->yawrate_buffer.push_back(-1 * imu.angular_velocity.z);
    }
  }

  rollrate_buffer_length = std::distance(angular_velocity_stop_status->rollrate_buffer.begin(), angular_velocity_stop_status->rollrate_buffer.end());
  pitchrate_buffer_length = std::distance(angular_velocity_stop_status->pitchrate_buffer.begin(), angular_velocity_stop_status->pitchrate_buffer.end());
  yawrate_buffer_length = std::distance(angular_velocity_stop_status->yawrate_buffer.begin(), angular_velocity_stop_status->yawrate_buffer.end());

  if (yawrate_buffer_length > angular_velocity_stop_parameter.estimated_number + estimated_time_buffer_num)
  {
    angular_velocity_stop_status->rollrate_buffer.erase(angular_velocity_stop_status->rollrate_buffer.begin());
    angular_velocity_stop_status->pitchrate_buffer.erase(angular_velocity_stop_status->pitchrate_buffer.begin());
    angular_velocity_stop_status->yawrate_buffer.erase(angular_velocity_stop_status->yawrate_buffer.begin());
  }

  if (velocity.twist.linear.x < angular_velocity_stop_parameter.stop_judgment_velocity_threshold)
  {
    ++angular_velocity_stop_status->stop_count;
  }
  else
  {
    angular_velocity_stop_status->stop_count = 0;
  }

  // mean
  if (angular_velocity_stop_status->stop_count > angular_velocity_stop_parameter.estimated_number + estimated_time_buffer_num)
  {
    roll_tmp = 0.0;
    pitch_tmp = 0.0;
    yaw_tmp = 0.0;
    for (i = 0; i < angular_velocity_stop_parameter.estimated_number; i++)
    {
      roll_tmp += angular_velocity_stop_status->rollrate_buffer[i];
      pitch_tmp += angular_velocity_stop_status->pitchrate_buffer[i];
      yaw_tmp += angular_velocity_stop_status->yawrate_buffer[i];
    }
    angular_velocity_offset_stop->rollrate_offset = -1 * roll_tmp / angular_velocity_stop_parameter.estimated_number;
    angular_velocity_offset_stop->pitchrate_offset = -1 * pitch_tmp / angular_velocity_stop_parameter.estimated_number;
    angular_velocity_offset_stop->yawrate_offset = -1 * yaw_tmp / angular_velocity_stop_parameter.estimated_number;
    angular_velocity_offset_stop->status.enabled_status = true;
    angular_velocity_offset_stop->status.estimate_status = true;
    angular_velocity_stop_status->estimate_start_status = true;
  }
  else
  {
    angular_velocity_offset_stop->rollrate_offset = angular_velocity_stop_status->rollrate_offset_stop_last;
    angular_velocity_offset_stop->pitchrate_offset = angular_velocity_stop_status->pitchrate_offset_stop_last;
    angular_velocity_offset_stop->yawrate_offset = angular_velocity_stop_status->yawrate_offset_stop_last;
    angular_velocity_offset_stop->status.estimate_status = false;
  }
  if (angular_velocity_stop_status->estimate_start_status == false)
  {
    angular_velocity_offset_stop->rollrate_offset = initial_angular_velocity_offset_stop;
    angular_velocity_offset_stop->pitchrate_offset = initial_angular_velocity_offset_stop;
    angular_velocity_offset_stop->yawrate_offset = initial_angular_velocity_offset_stop;
    angular_velocity_offset_stop->status.estimate_status = false;
    angular_velocity_offset_stop->status.enabled_status = false;
  }
  angular_velocity_stop_status->rollrate_offset_stop_last = angular_velocity_offset_stop->rollrate_offset;
  angular_velocity_stop_status->pitchrate_offset_stop_last = angular_velocity_offset_stop->pitchrate_offset;
  angular_velocity_stop_status->yawrate_offset_stop_last = angular_velocity_offset_stop->yawrate_offset;
}
