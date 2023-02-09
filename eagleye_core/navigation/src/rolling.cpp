// Copyright (c) 2022, Map IV, Inc.
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
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * rolling.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#define g 9.80665

void rolling_estimate(sensor_msgs::msg::Imu imu, geometry_msgs::msg::TwistStamped correction_velocity,
                      eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop, eagleye_msgs::msg::YawrateOffset yaw_rate_offset,
                      RollingParameter rolling_parameter, RollingStatus* rolling_status, eagleye_msgs::msg::Rolling* rolling)
{
  double acceleration_y;
  double velocity;
  double yaw_rate;

  double init_variance = 1.0;
  double acceleration_y_variance_negative;
  double acceleration_y_variance_positive;
  double update_gain;
  double filtered_acceleration_y;

  double in_sin;

  // Input data setup
  acceleration_y = imu.linear_acceleration.y;
  velocity = correction_velocity.twist.linear.x;

  if (std::abs(velocity) > rolling_parameter.stop_judgment_threshold)
  {
    yaw_rate = imu.angular_velocity.z + yaw_rate_offset.yaw_rate_offset;
  }
  else
  {
    yaw_rate = imu.angular_velocity.z + yaw_rate_offset_stop.yaw_rate_offset;
  }

  if (!rolling_status->data_status)
  {
    filtered_acceleration_y = acceleration_y;
    rolling_status->acceleration_y_last = acceleration_y;
    rolling_status->acceleration_y_variance_last = init_variance;
    rolling_status->data_status = true;
  }

  // Low Path Filter (about acceleration_y)
  if (rolling_status->data_status)
  {
    acceleration_y_variance_negative =
        rolling_status->acceleration_y_variance_last + rolling_parameter.filter_process_noise;
    update_gain = acceleration_y_variance_negative /
                  (acceleration_y_variance_negative + rolling_parameter.filter_observation_noise);
    filtered_acceleration_y =
        rolling_status->acceleration_y_last + update_gain * (acceleration_y - rolling_status->acceleration_y_last);
    acceleration_y_variance_positive = (1 - update_gain) * acceleration_y_variance_negative;

    rolling_status->acceleration_y_last = filtered_acceleration_y;
    rolling_status->acceleration_y_variance_last = acceleration_y_variance_positive;
  }

  // Rolling estimation
  rolling->header = imu.header;
  rolling->header.frame_id = "base_link";

  in_sin = (velocity * yaw_rate - filtered_acceleration_y) / g;

  if (std::abs(in_sin) < 1)
  {
    rolling->rolling_angle = std::asin(in_sin);
    rolling->status.enabled_status = true;
    rolling->status.estimate_status = true;
  }
  else if (rolling->status.estimate_status)
  {
    rolling->rolling_angle = rolling_status->rolling_last;
    rolling->status.enabled_status = false;
  }
  else
  {
    rolling->rolling_angle = 0;
    rolling->status.enabled_status = false;
    rolling->status.estimate_status = false;
  }

  rolling_status->rolling_last = rolling->rolling_angle;
}