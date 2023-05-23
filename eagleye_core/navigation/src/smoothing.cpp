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
 * smoothing.cpp
 * Author MapIV Takanose
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void smoothing_estimate(rtklib_msgs::RtklibNav rtklib_nav, geometry_msgs::TwistStamped velocity,SmoothingParameter smoothing_parameter,
  SmoothingStatus* smoothing_status,eagleye_msgs::Position* gnss_smooth_pos_enu)
{

  int i;
  double ecef_pos[3];
  double ecef_base_pos[3];
  double enu_pos[3];
  double gnss_smooth_pos[3] = {0};
  double sum_gnss_pos[3] = {0};
  bool gnss_update;
  std::size_t index_length;
  std::size_t time_buffer_length;
  std::size_t velocity_index_length;
  std::vector<int> velocity_index;
  std::vector<int> index;

  double estimated_buffer_number = smoothing_parameter.gnss_rate * smoothing_parameter.moving_average_time;

  if(gnss_smooth_pos_enu->ecef_base_pos.x == 0 && gnss_smooth_pos_enu->ecef_base_pos.y == 0 && gnss_smooth_pos_enu->ecef_base_pos.z == 0)
  {
    gnss_smooth_pos_enu->ecef_base_pos.x = rtklib_nav.ecef_pos.x;
    gnss_smooth_pos_enu->ecef_base_pos.y = rtklib_nav.ecef_pos.y;
    gnss_smooth_pos_enu->ecef_base_pos.z = rtklib_nav.ecef_pos.z;

    if(smoothing_parameter.ecef_base_pos_x != 0 && smoothing_parameter.ecef_base_pos_y != 0 && smoothing_parameter.ecef_base_pos_z != 0){
      gnss_smooth_pos_enu->ecef_base_pos.x = smoothing_parameter.ecef_base_pos_x;
      gnss_smooth_pos_enu->ecef_base_pos.y = smoothing_parameter.ecef_base_pos_y;
      gnss_smooth_pos_enu->ecef_base_pos.z = smoothing_parameter.ecef_base_pos_z;
    }
  }

  if(rtklib_nav.tow != 0){
    ecef_pos[0] = rtklib_nav.ecef_pos.x;
    ecef_pos[1] = rtklib_nav.ecef_pos.y;
    ecef_pos[2] = rtklib_nav.ecef_pos.z;
    ecef_base_pos[0] = gnss_smooth_pos_enu->ecef_base_pos.x;
    ecef_base_pos[1] = gnss_smooth_pos_enu->ecef_base_pos.y;
    ecef_base_pos[2] = gnss_smooth_pos_enu->ecef_base_pos.z;

    xyz2enu(ecef_pos, ecef_base_pos, enu_pos);

    if (!std::isfinite(enu_pos[0])||!std::isfinite(enu_pos[1])||!std::isfinite(enu_pos[2]))
    {
      enu_pos[0] = 0;
      enu_pos[1] = 0;
      enu_pos[2] = 0;
      gnss_update = false;
    }
    else
    {
      gnss_update = true;
    }
  }

  if(gnss_update == true){
    smoothing_status->time_buffer.push_back(rtklib_nav.header.stamp.toSec());
    smoothing_status->enu_pos_x_buffer.push_back(enu_pos[0]);
    smoothing_status->enu_pos_y_buffer.push_back(enu_pos[1]);
    smoothing_status->enu_pos_z_buffer.push_back(enu_pos[2]);
    smoothing_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);

    time_buffer_length = std::distance(smoothing_status->time_buffer.begin(), smoothing_status->time_buffer.end());

    if (time_buffer_length > estimated_buffer_number)
    {
      smoothing_status->time_buffer.erase(smoothing_status->time_buffer.begin());
      smoothing_status->enu_pos_x_buffer.erase(smoothing_status->enu_pos_x_buffer.begin());
      smoothing_status->enu_pos_y_buffer.erase(smoothing_status->enu_pos_y_buffer.begin());
      smoothing_status->enu_pos_z_buffer.erase(smoothing_status->enu_pos_z_buffer.begin());
      smoothing_status->correction_velocity_buffer.erase(smoothing_status->correction_velocity_buffer.begin());
    }

    if (smoothing_status->estimated_number < estimated_buffer_number)
    {
      ++smoothing_status->estimated_number;
      gnss_smooth_pos_enu->status.enabled_status = false;
    }
    else
    {
      smoothing_status->estimated_number = estimated_buffer_number;
      gnss_smooth_pos_enu->status.enabled_status = true;
    }

    if (smoothing_status->estimated_number == estimated_buffer_number){
      for (i = 0; i < smoothing_status->estimated_number; i++)
      {
        index.push_back(i);
        if (smoothing_status->correction_velocity_buffer[i] > smoothing_parameter.moving_judgement_threshold)
        {
          velocity_index.push_back(i);
        }
      }

      index_length = std::distance(index.begin(), index.end());
      velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

      for (i = 0; i < velocity_index_length; i++)
      {
        sum_gnss_pos[0] = sum_gnss_pos[0] + smoothing_status->enu_pos_x_buffer[velocity_index[i]];
        sum_gnss_pos[1] = sum_gnss_pos[1] + smoothing_status->enu_pos_y_buffer[velocity_index[i]];
        sum_gnss_pos[2] = sum_gnss_pos[2] + smoothing_status->enu_pos_z_buffer[velocity_index[i]];
      }

      if (velocity_index_length > index_length * smoothing_parameter.moving_ratio_threshold)
      {
        gnss_smooth_pos[0] = sum_gnss_pos[0]/velocity_index_length;
        gnss_smooth_pos[1] = sum_gnss_pos[1]/velocity_index_length;
        gnss_smooth_pos[2] = sum_gnss_pos[2]/velocity_index_length;
        gnss_smooth_pos_enu->status.estimate_status = true;
      }
    }
  }
  else
  {
    gnss_smooth_pos[0] = smoothing_status->last_pos[0];
    gnss_smooth_pos[1] = smoothing_status->last_pos[1];
    gnss_smooth_pos[2] = smoothing_status->last_pos[2];
    gnss_smooth_pos_enu->status.estimate_status = false;
  }

  smoothing_status->last_pos[0] = gnss_smooth_pos[0];
  smoothing_status->last_pos[1] = gnss_smooth_pos[1];
  smoothing_status->last_pos[2] = gnss_smooth_pos[2];

  gnss_smooth_pos_enu->enu_pos.x = gnss_smooth_pos[0];
  gnss_smooth_pos_enu->enu_pos.y = gnss_smooth_pos[1];
  gnss_smooth_pos_enu->enu_pos.z = gnss_smooth_pos[2];
  // gnss_smooth_pos_enu->ecef_base_pos = enu_absolute_pos.ecef_base_pos;
  gnss_smooth_pos_enu->header = rtklib_nav.header;
}
