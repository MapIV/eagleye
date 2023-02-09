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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void slip_angle_estimate(sensor_msgs::msg::Imu imu, geometry_msgs::msg::TwistStamped velocity, eagleye_msgs::msg::StatusStamped velocity_status,
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop, eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd, SlipangleParameter slip_angle_parameter,
  eagleye_msgs::msg::SlipAngle* slip_angle)
{

  int i;
  double doppler_slip;
  double yaw_rate;
  double acceleration_y;

  yaw_rate = imu.angular_velocity.z;

  if (std::abs(velocity.twist.linear.x) > slip_angle_parameter.stop_judgment_threshold)
  {
    yaw_rate = yaw_rate + yaw_rate_offset_2nd.yaw_rate_offset;
  }
  else
  {
    yaw_rate = yaw_rate + yaw_rate_offset_stop.yaw_rate_offset;
  }

  acceleration_y = velocity.twist.linear.x * yaw_rate;

  if (velocity_status.status.enabled_status == true && yaw_rate_offset_stop.status.enabled_status == true && yaw_rate_offset_2nd.status.enabled_status == true)
  {
      slip_angle->coefficient = slip_angle_parameter.manual_coefficient;
      slip_angle->slip_angle = slip_angle_parameter.manual_coefficient * acceleration_y;
      if (slip_angle_parameter.manual_coefficient != 0)
      {
        slip_angle->status.enabled_status = true;
      }
      else
      {
        slip_angle->status.enabled_status = false;
      }
      slip_angle->status.estimate_status = false;
  }
}
