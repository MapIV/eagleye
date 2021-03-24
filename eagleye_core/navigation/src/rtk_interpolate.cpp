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

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"
void rtk_interpolate_estimate(geometry_msgs::Vector3Stamped enu_vel, sensor_msgs::NavSatFix fix, PositionInterpolateParameter rtk_interpolate_parameter, PositionInterpolateStatus* rtk_interpolate_status, eagleye_msgs::Position* enu_absolute_rtk_interpolate,sensor_msgs::NavSatFix* eagleye_fix)
{

  double enu_pos[3],enu_rtk[3];
  double ecef_base_pos[3];
  double ecef_rtk[3];
  double llh_pos[3],llh_rtk[3];

  llh_rtk[0] = fix.latitude *M_PI/180;
  llh_rtk[1] = fix.longitude *M_PI/180;
  llh_rtk[2] = fix.altitude;

  llh2xyz(llh_rtk,ecef_rtk);

  if (enu_absolute_rtk_interpolate->ecef_base_pos.x == 0 && enu_absolute_rtk_interpolate->ecef_base_pos.y == 0 && enu_absolute_rtk_interpolate->ecef_base_pos.z == 0)
  {
    ecef_base_pos[0] = ecef_rtk[0];
    ecef_base_pos[1] = ecef_rtk[1];
    ecef_base_pos[2] = ecef_rtk[2];
    enu_absolute_rtk_interpolate->ecef_base_pos.x = ecef_rtk[0];
    enu_absolute_rtk_interpolate->ecef_base_pos.y = ecef_rtk[1];
    enu_absolute_rtk_interpolate->ecef_base_pos.z = ecef_rtk[2];
  }
  else
  {
    ecef_base_pos[0] = enu_absolute_rtk_interpolate->ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_rtk_interpolate->ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_rtk_interpolate->ecef_base_pos.z;
  }

  xyz2enu(ecef_rtk,ecef_base_pos,enu_rtk);

  if (rtk_interpolate_status->position_stamp_last != fix.header.stamp.toSec() && fix.status.status == 0)
  {
    rtk_interpolate_status->provisional_enu_pos_x = enu_rtk[0];
    rtk_interpolate_status->provisional_enu_pos_y = enu_rtk[1];
    rtk_interpolate_status->provisional_enu_pos_z = enu_rtk[2];
    enu_absolute_rtk_interpolate->status.enabled_status = true;
    enu_absolute_rtk_interpolate->status.estimate_status = true;
  }
  else if(rtk_interpolate_status->time_last != 0 && sqrt((enu_vel.vector.x * enu_vel.vector.x) + (enu_vel.vector.y * enu_vel.vector.y) + (enu_vel.vector.z * enu_vel.vector.z)) > rtk_interpolate_parameter.stop_judgment_velocity_threshold)
  {
    rtk_interpolate_status->provisional_enu_pos_x = enu_absolute_rtk_interpolate->enu_pos.x + enu_vel.vector.x * (enu_vel.header.stamp.toSec() - rtk_interpolate_status->time_last);
    rtk_interpolate_status->provisional_enu_pos_y = enu_absolute_rtk_interpolate->enu_pos.y + enu_vel.vector.y * (enu_vel.header.stamp.toSec() - rtk_interpolate_status->time_last);
    rtk_interpolate_status->provisional_enu_pos_z = enu_absolute_rtk_interpolate->enu_pos.z + enu_vel.vector.z * (enu_vel.header.stamp.toSec() - rtk_interpolate_status->time_last);
    enu_absolute_rtk_interpolate->status.enabled_status = true;
    enu_absolute_rtk_interpolate->status.estimate_status = false;
  }

  enu_pos[0] = rtk_interpolate_status->provisional_enu_pos_x;
  enu_pos[1] = rtk_interpolate_status->provisional_enu_pos_y;
  enu_pos[2] = rtk_interpolate_status->provisional_enu_pos_z;

  enu2llh(enu_pos, ecef_base_pos, llh_pos);

  eagleye_fix->latitude = llh_pos[0] * 180/M_PI;
  eagleye_fix->longitude = llh_pos[1] * 180/M_PI;
  eagleye_fix->altitude = llh_pos[2];

  enu_absolute_rtk_interpolate->enu_pos.x = enu_pos[0];
  enu_absolute_rtk_interpolate->enu_pos.y = enu_pos[1];
  enu_absolute_rtk_interpolate->enu_pos.z = enu_pos[2];

  rtk_interpolate_status->time_last = enu_vel.header.stamp.toSec();
  rtk_interpolate_status->position_stamp_last = fix.header.stamp.toSec();
}
