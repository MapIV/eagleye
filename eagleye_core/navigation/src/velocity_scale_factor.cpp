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
 * velocity_scale_factor.cpp
 * Author MapIV Sekino
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void velocity_scale_factor_estimate(const rtklib_msgs::RtklibNav rtklib_nav, const geometry_msgs::TwistStamped velocity, const VelocityScaleFactorParameter velocity_scale_factor_parameter, VelocityScaleFactorStatus* velocity_scale_factor_status,eagleye_msgs::VelocityScaleFactor* velocity_scale_factor)
{
    double ecef_vel[3];
    double ecef_pos[3];
    double enu_vel[3];

    bool gnss_status;
    int i;
    double initial_velocity_scale_factor = 1.0;
    double doppler_velocity = 0.0;
    double raw_velocity_scale_factor = 0.0;
    std::size_t index_length;
    std::size_t gnss_status_buffer_length;

    ecef_vel[0] = rtklib_nav.ecef_vel.x;
    ecef_vel[1] = rtklib_nav.ecef_vel.y;
    ecef_vel[2] = rtklib_nav.ecef_vel.z;
    ecef_pos[0] = rtklib_nav.ecef_pos.x;
    ecef_pos[1] = rtklib_nav.ecef_pos.y;
    ecef_pos[2] = rtklib_nav.ecef_pos.z;

    xyz2enu_vel(ecef_vel, ecef_pos, enu_vel);

    doppler_velocity = sqrt((enu_vel[0] * enu_vel[0]) + (enu_vel[1] * enu_vel[1]) + (enu_vel[2] * enu_vel[2]));

  if (velocity_scale_factor_status->estimated_number < velocity_scale_factor_parameter.estimated_number_max)
  {
    ++velocity_scale_factor_status->estimated_number;
  }
  else
  {
    velocity_scale_factor_status->estimated_number = velocity_scale_factor_parameter.estimated_number_max;
  }

  if (velocity_scale_factor_status->tow_last == rtklib_nav.tow)
  {
    gnss_status = false;
    doppler_velocity = 0;
    velocity_scale_factor_status->tow_last = rtklib_nav.tow;
  }
  else
  {
    gnss_status = true;
    doppler_velocity = doppler_velocity;
    velocity_scale_factor_status->tow_last = rtklib_nav.tow;
  }

  velocity_scale_factor_status->gnss_status_buffer.push_back(gnss_status);
  velocity_scale_factor_status->doppler_velocity_buffer.push_back(doppler_velocity);
  velocity_scale_factor_status->velocity_buffer.push_back(velocity.twist.linear.x);

  gnss_status_buffer_length = std::distance(velocity_scale_factor_status->gnss_status_buffer.begin(), velocity_scale_factor_status->gnss_status_buffer.end());

  if (gnss_status_buffer_length > velocity_scale_factor_parameter.estimated_number_max)
  {
    velocity_scale_factor_status->gnss_status_buffer.erase(velocity_scale_factor_status->gnss_status_buffer.begin());
    velocity_scale_factor_status->doppler_velocity_buffer.erase(velocity_scale_factor_status->doppler_velocity_buffer.begin());
    velocity_scale_factor_status->velocity_buffer.erase(velocity_scale_factor_status->velocity_buffer.begin());
  }

  std::vector<int> gnss_index;
  std::vector<int> velocity_index;
  std::vector<int> index;
  std::vector<double> velocity_scale_factor_buffer;

  if (velocity_scale_factor_status->estimated_number > velocity_scale_factor_parameter.estimated_number_min && velocity_scale_factor_status->gnss_status_buffer[velocity_scale_factor_status->estimated_number - 1] == true && velocity_scale_factor_status->velocity_buffer[velocity_scale_factor_status->estimated_number - 1] > velocity_scale_factor_parameter.estimated_velocity_threshold)
  {
    for (i = 0; i < velocity_scale_factor_status->estimated_number; i++)
    {
      if (velocity_scale_factor_status->gnss_status_buffer[i] == true)
      {
        gnss_index.push_back(i);
      }
      if (velocity_scale_factor_status->velocity_buffer[i] > velocity_scale_factor_parameter.estimated_velocity_threshold)
      {
        velocity_index.push_back(i);
      }
    }

    set_intersection(gnss_index.begin(), gnss_index.end(), velocity_index.begin(), velocity_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > velocity_scale_factor_status->estimated_number * velocity_scale_factor_parameter.estimated_coefficient)
    {
      for (i = 0; i < index_length; i++)
      {
        velocity_scale_factor_buffer.push_back(velocity_scale_factor_status->doppler_velocity_buffer[index[i]] / velocity_scale_factor_status->velocity_buffer[index[i]]);
      }

      velocity_scale_factor->status.estimate_status = true;
      velocity_scale_factor_status->estimate_start_status = true;
    }
    else
    {
      velocity_scale_factor->status.estimate_status = false;
    }
  }
  else
  {
    velocity_scale_factor->status.estimate_status = false;
  }

  if (velocity_scale_factor->status.estimate_status == true)
  {
    // median
    size_t size = velocity_scale_factor_buffer.size();
    double* t = new double[size];
    std::copy(velocity_scale_factor_buffer.begin(), velocity_scale_factor_buffer.end(), t);
    std::sort(t, &t[size]);
    raw_velocity_scale_factor = size % 2 ? t[size / 2] : (t[(size / 2) - 1] + t[size / 2]) / 2;
    delete[] t;
    velocity_scale_factor->scale_factor = raw_velocity_scale_factor;
  }
  else if (velocity_scale_factor->status.estimate_status == false)
  {
    raw_velocity_scale_factor = 0;
    velocity_scale_factor->scale_factor = velocity_scale_factor_status->velocity_scale_factor_last;
  }

  if (velocity_scale_factor_status->estimate_start_status == true)
  {
    velocity_scale_factor->status.enabled_status = true;
    velocity_scale_factor->correction_velocity.linear.x = velocity.twist.linear.x * velocity_scale_factor->scale_factor;
  }
  else
  {
    velocity_scale_factor->status.enabled_status = false;
    velocity_scale_factor->scale_factor = initial_velocity_scale_factor;
    velocity_scale_factor->correction_velocity.linear.x = velocity.twist.linear.x * initial_velocity_scale_factor;
  }

  velocity_scale_factor_status->velocity_scale_factor_last = velocity_scale_factor->scale_factor;

}
