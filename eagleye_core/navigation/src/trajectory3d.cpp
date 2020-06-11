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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "navigation.hpp"

void trajectory3d_estimate(const sensor_msgs::Imu imu, const eagleye_msgs::VelocityScaleFactor velocity_scale_factor, const eagleye_msgs::Heading heading_interpolate_3rd, const eagleye_msgs::YawrateOffset yawrate_offset_stop, const eagleye_msgs::YawrateOffset yawrate_offset_2nd, const eagleye_msgs::Pitching pitching, const TrajectoryParameter trajectory_parameter, TrajectoryStatus* trajectory_status, geometry_msgs::Vector3Stamped* enu_vel, eagleye_msgs::Position* enu_relative_pos, geometry_msgs::TwistStamped* eagleye_twist)
{

  if (trajectory_parameter.reverse_imu == false)
  {
    if (velocity_scale_factor.correction_velocity.linear.x > trajectory_parameter.stop_judgment_velocity_threshold && yawrate_offset_2nd.status.enabled_status == true)
    {
      eagleye_twist->twist.angular.z = -1 * imu.angular_velocity.z + yawrate_offset_2nd.yawrate_offset; //Inverted because the coordinate system is reversed
    }
    else
    {
      eagleye_twist->twist.angular.z = -1 * imu.angular_velocity.z + yawrate_offset_stop.yawrate_offset; //Inverted because the coordinate system is reversed
    }
  }
  else if (velocity_scale_factor.correction_velocity.linear.x > trajectory_parameter.stop_judgment_velocity_threshold && yawrate_offset_2nd.status.enabled_status == true)
  {
    if (velocity_scale_factor.correction_velocity.linear.x > trajectory_parameter.stop_judgment_velocity_threshold)
    {
      eagleye_twist->twist.angular.z = -1 * (-1 * imu.angular_velocity.z) + yawrate_offset_2nd.yawrate_offset; //Inverted because the coordinate system is reversed
    }
    else
    {
      eagleye_twist->twist.angular.z = -1 * (-1 * imu.angular_velocity.z) + yawrate_offset_stop.yawrate_offset; //Inverted because the coordinate system is reversed
    }
  }
  eagleye_twist->twist.linear.x = velocity_scale_factor.correction_velocity.linear.x;

  if (trajectory_status->estimate_status_count == 0 && velocity_scale_factor.status.enabled_status == true && heading_interpolate_3rd.status.enabled_status == true)
  {
    trajectory_status->estimate_status_count = 1;
  }
  else if (trajectory_status->estimate_status_count == 1)
  {
    trajectory_status->estimate_status_count = 2;
  }

  if (trajectory_status->estimate_status_count == 2)
  {
    enu_vel->vector.x = sin(heading_interpolate_3rd.heading_angle) * cos(pitching.pitching_angle) * velocity_scale_factor.correction_velocity.linear.x; //vel_e
    enu_vel->vector.y = cos(heading_interpolate_3rd.heading_angle) * cos(pitching.pitching_angle) * velocity_scale_factor.correction_velocity.linear.x; //vel_n
    enu_vel->vector.z = sin(pitching.pitching_angle) * velocity_scale_factor.correction_velocity.linear.x; //vel_u
  }

  if (trajectory_status->estimate_status_count == 2 && velocity_scale_factor.correction_velocity.linear.x > 0 && trajectory_status->time_last != 0)
  {
    enu_relative_pos->enu_pos.x = enu_relative_pos->enu_pos.x + enu_vel->vector.x * (imu.header.stamp.toSec() - trajectory_status->time_last);
    enu_relative_pos->enu_pos.y = enu_relative_pos->enu_pos.y + enu_vel->vector.y * (imu.header.stamp.toSec() - trajectory_status->time_last);
    enu_relative_pos->enu_pos.z = enu_relative_pos->enu_pos.z + enu_vel->vector.z * (imu.header.stamp.toSec() - trajectory_status->time_last);
    enu_relative_pos->status.enabled_status = enu_relative_pos->status.estimate_status = true;
  }

  trajectory_status->time_last = imu.header.stamp.toSec();
}
