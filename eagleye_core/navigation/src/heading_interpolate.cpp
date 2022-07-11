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
 * heading_interpolate.cpp
 * Author MapIV Sekino
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void heading_interpolate_estimate(const sensor_msgs::Imu imu, const geometry_msgs::TwistStamped velocity,
  const eagleye_msgs::YawrateOffset yawrate_offset_stop,const eagleye_msgs::YawrateOffset yawrate_offset,const eagleye_msgs::Heading heading,
  const eagleye_msgs::SlipAngle slip_angle,const HeadingInterpolateParameter heading_interpolate_parameter, HeadingInterpolateStatus* heading_interpolate_status,
  eagleye_msgs::Heading* heading_interpolate)
{
  int i;
  int estimate_index = 0;
  double yawrate = 0.0;
  double diff_estimate_heading_angle = 0.0;
  bool heading_estimate_status;
  std::size_t imu_stamp_buffer_length;

  yawrate = imu.angular_velocity.z;

  if (std::abs(velocity.twist.linear.x) > heading_interpolate_parameter.stop_judgment_velocity_threshold)
  {
    yawrate = yawrate + yawrate_offset.yawrate_offset;
  }
  else
  {
    yawrate = yawrate + yawrate_offset_stop.yawrate_offset;
  }

  if (heading_interpolate_status->number_buffer < heading_interpolate_parameter.number_buffer_max)
  {
    ++heading_interpolate_status->number_buffer;
  }
  else
  {
    heading_interpolate_status->number_buffer = heading_interpolate_parameter.number_buffer_max;
  }

  if (heading_interpolate_status->heading_stamp_last != heading.header.stamp.toSec() && heading.status.estimate_status == true)
  {
    heading_estimate_status = true;
    heading_interpolate_status->heading_estimate_start_status = true;
    ++heading_interpolate_status->heading_estimate_status_count;
  }
  else
  {
    heading_estimate_status = false;
  }

  if(heading_interpolate_status->time_last != 0 && std::abs(velocity.twist.linear.x) >
    heading_interpolate_parameter.stop_judgment_velocity_threshold)
  {
    heading_interpolate_status->provisional_heading_angle = heading_interpolate_status->provisional_heading_angle +
      (yawrate * (imu.header.stamp.toSec() - heading_interpolate_status->time_last));
  }

  // data buffer generate
  heading_interpolate_status->provisional_heading_angle_buffer.push_back(heading_interpolate_status->provisional_heading_angle);
  heading_interpolate_status->imu_stamp_buffer.push_back(imu.header.stamp.toSec());
  imu_stamp_buffer_length = std::distance(heading_interpolate_status->imu_stamp_buffer.begin(), heading_interpolate_status->imu_stamp_buffer.end());

  if (imu_stamp_buffer_length > heading_interpolate_parameter.number_buffer_max)
  {
    heading_interpolate_status->provisional_heading_angle_buffer.erase(heading_interpolate_status->provisional_heading_angle_buffer .begin());
    heading_interpolate_status->imu_stamp_buffer.erase(heading_interpolate_status->imu_stamp_buffer .begin());
  }

  if (heading_interpolate_status->heading_estimate_start_status == true)
  {
    if (heading_estimate_status == true)
    {
      for (estimate_index = heading_interpolate_status->number_buffer; estimate_index > 0; estimate_index--)
      {
        if (heading_interpolate_status->imu_stamp_buffer[estimate_index-1] == heading.header.stamp.toSec())
        {
          break;
        }
      }
    }

    if (heading_estimate_status == true && estimate_index > 0 && heading_interpolate_status->number_buffer >= estimate_index &&
      heading_interpolate_status->heading_estimate_status_count > 1)
    {
      diff_estimate_heading_angle = (heading_interpolate_status->provisional_heading_angle_buffer[estimate_index-1] - heading.heading_angle);
      for (i = estimate_index; i <= heading_interpolate_status->number_buffer; i++)
      {
        heading_interpolate_status->provisional_heading_angle_buffer[i-1] = heading_interpolate_status->provisional_heading_angle_buffer[i-1] -
          diff_estimate_heading_angle;
      }
      heading_interpolate_status->provisional_heading_angle =
        heading_interpolate_status->provisional_heading_angle_buffer[heading_interpolate_status->number_buffer-1];

      heading_interpolate->status.enabled_status = true;
      heading_interpolate->status.estimate_status = true;
    }
    else if (heading_interpolate_status->heading_estimate_status_count == 1)
    {
      heading_interpolate_status->provisional_heading_angle = heading.heading_angle;
      heading_interpolate->status.enabled_status = true;
      heading_interpolate->status.estimate_status = false;
    }
    else
    {
      heading_interpolate->status.estimate_status = false;
    }
  }

  if (heading_interpolate_status->heading_estimate_start_status == true)
  {
    heading_interpolate->heading_angle = heading_interpolate_status->provisional_heading_angle + slip_angle.slip_angle;
  }
  else
  {
    heading_interpolate->heading_angle = 0.0;
    heading_interpolate->status.enabled_status = false;
    heading_interpolate->status.estimate_status = false;
  }

  heading_interpolate_status->time_last = imu.header.stamp.toSec();
  heading_interpolate_status->heading_stamp_last = heading.header.stamp.toSec();

}
