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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void heading_interpolate_estimate(const sensor_msgs::msg::Imu imu, const geometry_msgs::msg::TwistStamped velocity,
  const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop, const eagleye_msgs::msg::YawrateOffset yaw_rate_offset, const eagleye_msgs::msg::Heading heading,
  const eagleye_msgs::msg::SlipAngle slip_angle,const HeadingInterpolateParameter heading_interpolate_parameter, HeadingInterpolateStatus* heading_interpolate_status,
  eagleye_msgs::msg::Heading* heading_interpolate)
{
  int i;
  int estimate_index = 0;
  double yaw_rate = 0.0;
  double diff_estimate_heading_angle = 0.0;
  bool heading_estimate_status;
  std::size_t imu_stamp_buffer_length;

  double proc_noise = heading_interpolate_parameter.proc_noise;

  double search_buffer_number = heading_interpolate_parameter.sync_search_period * heading_interpolate_parameter.imu_rate;

  rclcpp::Time ros_clock(heading.header.stamp);
  rclcpp::Time ros_clock2(imu.header.stamp);
  auto heading_time = ros_clock.seconds();
  auto imu_time = ros_clock2.seconds();

  yaw_rate = imu.angular_velocity.z;

  if (std::abs(velocity.twist.linear.x) > heading_interpolate_parameter.stop_judgment_threshold)
  {
    yaw_rate = yaw_rate + yaw_rate_offset.yaw_rate_offset;
  }
  else
  {
    yaw_rate = yaw_rate + yaw_rate_offset_stop.yaw_rate_offset;
  }

  if (heading_interpolate_status->number_buffer < search_buffer_number)
  {
    ++heading_interpolate_status->number_buffer;
  }
  else
  {
    heading_interpolate_status->number_buffer = search_buffer_number;
  }

  if (heading_interpolate_status->heading_stamp_last != heading_time && heading.status.estimate_status == true)
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
    heading_interpolate_parameter.stop_judgment_threshold)
  {
    heading_interpolate_status->provisional_heading_angle = heading_interpolate_status->provisional_heading_angle + (yaw_rate * (imu_time - heading_interpolate_status->time_last));
    heading_interpolate_status->heading_variance_last += proc_noise*proc_noise;
  }

  // data buffer generate
  heading_interpolate_status->provisional_heading_angle_buffer.push_back(heading_interpolate_status->provisional_heading_angle);
  heading_interpolate_status->imu_stamp_buffer.push_back(imu_time);
  imu_stamp_buffer_length = std::distance(heading_interpolate_status->imu_stamp_buffer.begin(), heading_interpolate_status->imu_stamp_buffer.end());

  if (imu_stamp_buffer_length > search_buffer_number)
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
        if (heading_interpolate_status->imu_stamp_buffer[estimate_index-1] == heading_time)
        {
          break;
        }
      }
    }

    if (heading_estimate_status == true && estimate_index > 0 && heading_interpolate_status->number_buffer >= estimate_index &&
      heading_interpolate_status->heading_estimate_status_count > 1)
    {
      double heading_variance = heading.variance;
      diff_estimate_heading_angle = (heading_interpolate_status->provisional_heading_angle_buffer[estimate_index-1] - heading.heading_angle);
      for (i = estimate_index; i <= heading_interpolate_status->number_buffer; i++)
      {
        heading_interpolate_status->provisional_heading_angle_buffer[i-1] = heading_interpolate_status->provisional_heading_angle_buffer[i-1] -
          diff_estimate_heading_angle;
        heading_variance += proc_noise*proc_noise;
      }
      heading_interpolate_status->provisional_heading_angle =
        heading_interpolate_status->provisional_heading_angle_buffer[heading_interpolate_status->number_buffer-1];

      heading_interpolate->status.enabled_status = true;
      heading_interpolate->status.estimate_status = true;
      heading_interpolate_status->heading_variance_last = heading_variance;
    }
    else if (heading_interpolate_status->heading_estimate_status_count == 1)
    {
      heading_interpolate_status->provisional_heading_angle = heading.heading_angle;
      heading_interpolate->status.enabled_status = true;
      heading_interpolate->status.estimate_status = false;
      heading_interpolate_status->heading_variance_last = heading.variance;
    }
    else
    {
      heading_interpolate->status.estimate_status = false;
    }
  }

  if (heading_interpolate_status->heading_estimate_start_status == true)
  {
    heading_interpolate->heading_angle = heading_interpolate_status->provisional_heading_angle + slip_angle.slip_angle;
    heading_interpolate->variance = heading_interpolate_status->heading_variance_last;
  }
  else
  {
    heading_interpolate->heading_angle = 0.0;
    heading_interpolate->status.enabled_status = false;
    heading_interpolate->status.estimate_status = false;
  }

  heading_interpolate_status->time_last = imu_time;
  heading_interpolate_status->heading_stamp_last = heading_time;

}
