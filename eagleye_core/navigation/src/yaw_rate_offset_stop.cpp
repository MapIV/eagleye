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
 * yaw_rate_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void yaw_rate_offset_stop_estimate(const geometry_msgs::msg::TwistStamped velocity, const sensor_msgs::msg::Imu imu, const YawrateOffsetStopParameter yaw_rate_offset_stop_parameter, YawrateOffsetStopStatus* yaw_rate_offset_stop_status, eagleye_msgs::msg::YawrateOffset* yaw_rate_offset_stop)
{

  int i;
  double tmp = 0.0;
  double initial_yaw_rate_offset_stop = 0.0;
  std::size_t yaw_rate_buffer_length;

  double estimated_buffer_number = yaw_rate_offset_stop_parameter.imu_rate * yaw_rate_offset_stop_parameter.estimated_interval;
  double estimated_time_buffer_number = estimated_buffer_number; //TODO rename

  // data buffer generate
  if (yaw_rate_offset_stop_status->estimate_start_status == false)
  {
    yaw_rate_offset_stop_status->yaw_rate_buffer.push_back(imu.angular_velocity.z);
  }
  else if ( std::fabs(std::fabs(yaw_rate_offset_stop_status->yaw_rate_offset_stop_last) - std::fabs(imu.angular_velocity.z)) < yaw_rate_offset_stop_parameter.outlier_threshold && yaw_rate_offset_stop_status->estimate_start_status == true)
  {
    yaw_rate_offset_stop_status->yaw_rate_buffer.push_back(imu.angular_velocity.z);
  }

  yaw_rate_buffer_length = std::distance(yaw_rate_offset_stop_status->yaw_rate_buffer.begin(), yaw_rate_offset_stop_status->yaw_rate_buffer.end());

  if (yaw_rate_buffer_length > estimated_buffer_number + estimated_time_buffer_number)
  {
    yaw_rate_offset_stop_status->yaw_rate_buffer.erase(yaw_rate_offset_stop_status->yaw_rate_buffer.begin());
  }

  if (velocity.twist.linear.x < yaw_rate_offset_stop_parameter.stop_judgment_threshold)
  {
    ++yaw_rate_offset_stop_status->stop_count;
  }
  else
  {
    yaw_rate_offset_stop_status->stop_count = 0;
  }

  // mean
  if (yaw_rate_offset_stop_status->stop_count > estimated_buffer_number + estimated_time_buffer_number)
  {
    tmp = 0.0;
    for (i = 0; i < estimated_buffer_number; i++)
    {
      tmp += yaw_rate_offset_stop_status->yaw_rate_buffer[i];
    }
    yaw_rate_offset_stop->yaw_rate_offset = -1 * tmp / estimated_buffer_number;
    yaw_rate_offset_stop->status.enabled_status = true;
    yaw_rate_offset_stop->status.estimate_status = true;
    yaw_rate_offset_stop_status->estimate_start_status = true;
  }
  else
  {
    yaw_rate_offset_stop->yaw_rate_offset = yaw_rate_offset_stop_status->yaw_rate_offset_stop_last;
    yaw_rate_offset_stop->status.estimate_status = false;
  }
  if (yaw_rate_offset_stop_status->estimate_start_status == false)
  {
    yaw_rate_offset_stop->yaw_rate_offset = initial_yaw_rate_offset_stop;
    yaw_rate_offset_stop->status.estimate_status = false;
    yaw_rate_offset_stop->status.enabled_status = false;
  }
  yaw_rate_offset_stop_status->yaw_rate_offset_stop_last = yaw_rate_offset_stop->yaw_rate_offset;
}
