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
 * yawrate_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void yawrate_offset_stop_estimate(const geometry_msgs::TwistStamped velocity, const sensor_msgs::Imu imu,
  const YawrateOffsetStopParameter yawrate_offset_stop_parameter, YawrateOffsetStopStatus* yawrate_offset_stop_status,
  eagleye_msgs::YawrateOffset* yawrate_offset_stop)
{

  int i;
  double tmp = 0.0;
  double initial_yawrate_offset_stop = 0.0;
  std::size_t yawrate_buffer_length;

  double estimated_buffer_number = yawrate_offset_stop_parameter.imu_rate * yawrate_offset_stop_parameter.estimated_interval;
  double estimated_time_buffer_number = estimated_buffer_number; //TODO rename


  // std::cout<< "estimated_number: " << _yawrate_offset_stop_parameter.estimated_number << std::endl;


  // data buffer generate
  if (yawrate_offset_stop_status->estimate_start_status == false)
  {
    yawrate_offset_stop_status->yawrate_buffer.push_back(imu.angular_velocity.z);
  }
  else if ( std::fabs(std::fabs(yawrate_offset_stop_status->yawrate_offset_stop_last) - std::fabs(imu.angular_velocity.z)) <
    yawrate_offset_stop_parameter.outlier_threshold && yawrate_offset_stop_status->estimate_start_status == true)
  {
    yawrate_offset_stop_status->yawrate_buffer.push_back(imu.angular_velocity.z);
  }

  yawrate_buffer_length = std::distance(yawrate_offset_stop_status->yawrate_buffer.begin(), yawrate_offset_stop_status->yawrate_buffer.end());

  if (yawrate_buffer_length > estimated_buffer_number + estimated_time_buffer_number)
  {
    yawrate_offset_stop_status->yawrate_buffer.erase(yawrate_offset_stop_status->yawrate_buffer.begin());
  }

  if (velocity.twist.linear.x < yawrate_offset_stop_parameter.stop_judgment_threshold)
  {
    ++yawrate_offset_stop_status->stop_count;
  }
  else
  {
    yawrate_offset_stop_status->stop_count = 0;
  }

  // mean
  if (yawrate_offset_stop_status->stop_count > estimated_buffer_number + estimated_time_buffer_number)
  {
    tmp = 0.0;
    for (i = 0; i < estimated_buffer_number; i++)
    {
      tmp += yawrate_offset_stop_status->yawrate_buffer[i];
    }
    yawrate_offset_stop->yawrate_offset = -1 * tmp / estimated_buffer_number;
    yawrate_offset_stop->status.enabled_status = true;
    yawrate_offset_stop->status.estimate_status = true;
    yawrate_offset_stop_status->estimate_start_status = true;
  }
  else
  {
    yawrate_offset_stop->yawrate_offset = yawrate_offset_stop_status->yawrate_offset_stop_last;
    yawrate_offset_stop->status.estimate_status = false;
  }
  if (yawrate_offset_stop_status->estimate_start_status == false)
  {
    yawrate_offset_stop->yawrate_offset = initial_yawrate_offset_stop;
    yawrate_offset_stop->status.estimate_status = false;
    yawrate_offset_stop->status.enabled_status = false;
  }
  yawrate_offset_stop_status->yawrate_offset_stop_last = yawrate_offset_stop->yawrate_offset;
}
