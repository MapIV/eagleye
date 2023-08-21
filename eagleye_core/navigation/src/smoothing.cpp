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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

// This function calculates the moving average for each of the XYZ components in the ENU coordinate system. 
// Only the positions when the vehicle is in motion are stored in the queue. Until the queue is filled, it outputs 0. 
// The `estimate_status` indicates whether the ECEF of rtklib_nav is valid or not, 
// and the `enabled_status` indicates whether the queue is fulfilled and the moving average is generated.
void smoothing_estimate(rtklib_msgs::msg::RtklibNav rtklib_nav, geometry_msgs::msg::TwistStamped velocity,
  SmoothingParameter smoothing_parameter, SmoothingStatus* smoothing_status,eagleye_msgs::msg::Position* gnss_smooth_pos_enu)
{

  int i;
  double ecef_pos[3];
  double ecef_base_pos[3];
  double enu_pos[3];
  double gnss_smooth_pos[3] = {0};
  double sum_gnss_pos[3] = {0};
  bool gnss_update = false;
  std::size_t index_length;
  std::size_t velocity_index_length;
  std::vector<int> velocity_index;
  std::vector<int> index;

  const double estimated_buffer_number = smoothing_parameter.gnss_rate * smoothing_parameter.moving_average_time;
  const auto rtklib_nav_time = rclcpp::Time(rtklib_nav.header.stamp).seconds();

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

  // Convert ECEF to ENU coordinates
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
    // If the vehicle is moving, push the values to queue
    if (velocity.twist.linear.x > smoothing_parameter.moving_judgment_threshold)
    {
      smoothing_status->time_buffer.push_back(rtklib_nav_time);
      smoothing_status->enu_pos_x_buffer.push_back(enu_pos[0]);
      smoothing_status->enu_pos_y_buffer.push_back(enu_pos[1]);
      smoothing_status->enu_pos_z_buffer.push_back(enu_pos[2]);
      smoothing_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);
      ++smoothing_status->estimated_number;
    }

    // Pop the oldest value from queue
    if (smoothing_status->time_buffer.size() > estimated_buffer_number)
    {
      smoothing_status->time_buffer.erase(smoothing_status->time_buffer.begin());
      smoothing_status->enu_pos_x_buffer.erase(smoothing_status->enu_pos_x_buffer.begin());
      smoothing_status->enu_pos_y_buffer.erase(smoothing_status->enu_pos_y_buffer.begin());
      smoothing_status->enu_pos_z_buffer.erase(smoothing_status->enu_pos_z_buffer.begin());
      smoothing_status->correction_velocity_buffer.erase(smoothing_status->correction_velocity_buffer.begin());
    }

    // If the queue has more than the specified number of values, the enabled_status is set to true
    if (smoothing_status->estimated_number < estimated_buffer_number)
    {
      gnss_smooth_pos_enu->status.enabled_status = false;
    }
    else
    {
      smoothing_status->estimated_number = estimated_buffer_number;
      gnss_smooth_pos_enu->status.enabled_status = true;
    }

    // If queue is fulfilled, compute a moving average of the position
    if (smoothing_status->estimated_number == estimated_buffer_number){
      for (i = 0; i < smoothing_status->estimated_number; i++)
      {
        // In the past, index_length means the total length of the queue, and velocity_index_length means the number of elements that have a certain velocity.
        // Now all elements of `smoothing_status->correction_velocity_buffer` are greater than `smoothing_parameter.moving_judgment_threshold`
        index.push_back(i);
        velocity_index.push_back(i);
      }

      index_length = std::distance(index.begin(), index.end());
      velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

      for (i = 0; i < velocity_index_length; i++)
      {
        sum_gnss_pos[0] = sum_gnss_pos[0] + smoothing_status->enu_pos_x_buffer[velocity_index[i]];
        sum_gnss_pos[1] = sum_gnss_pos[1] + smoothing_status->enu_pos_y_buffer[velocity_index[i]];
        sum_gnss_pos[2] = sum_gnss_pos[2] + smoothing_status->enu_pos_z_buffer[velocity_index[i]];
      }

      // In the past, the averaged position was used only when the number of elements with velocity was above the certain ratio.
      // `velocity_index_length > index_length * smoothing_parameter.moving_ratio_threshold`
      // Now, since all elements in the queue have velocities, the average position is always available.
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
    // If rtklib_nav is not enabled, use the last values
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
