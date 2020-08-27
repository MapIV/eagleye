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

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void slip_angle_estimate(sensor_msgs::Imu imu, eagleye_msgs::VelocityScaleFactor velocity_scale_factor, eagleye_msgs::YawrateOffset yawrate_offset_stop, eagleye_msgs::YawrateOffset yawrate_offset_2nd, SlipangleParameter slip_angle_parameter,eagleye_msgs::SlipAngle* slip_angle)
{

  int i;
  double doppler_slip;
  double yawrate;
  double acceleration_y;

  if (slip_angle_parameter.reverse_imu == false)
  {
    yawrate = imu.angular_velocity.z;
  }
  else if (slip_angle_parameter.reverse_imu == true)
  {
    yawrate = -1 * imu.angular_velocity.z;
  }

  if (std::abs(velocity_scale_factor.correction_velocity.linear.x) > slip_angle_parameter.stop_judgment_velocity_threshold)
  {
    yawrate = yawrate + yawrate_offset_2nd.yawrate_offset;
  }
  else
  {
    yawrate = yawrate + yawrate_offset_stop.yawrate_offset;
  }

  acceleration_y = velocity_scale_factor.correction_velocity.linear.x * yawrate;

  if (velocity_scale_factor.status.enabled_status == true && yawrate_offset_stop.status.enabled_status == true && yawrate_offset_2nd.status.enabled_status == true)
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
