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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#define knot2mps 0.51477

void velocity_scale_factor_estimate_(const geometry_msgs::msg::TwistStamped velocity, const VelocityScaleFactorParameter velocity_scale_factor_parameter,
  VelocityScaleFactorStatus* velocity_scale_factor_status, geometry_msgs::msg::TwistStamped* correction_velocity,
  eagleye_msgs::msg::VelocityScaleFactor* velocity_scale_factor)
{ 
    int i;
    double initial_velocity_scale_factor = 1.0;
    double raw_velocity_scale_factor = 0.0;
    std::size_t index_length;
    std::size_t gnss_status_buffer_length;
    double estimated_number_cur;

    double estimated_buffer_number_min = velocity_scale_factor_parameter.estimated_minimum_interval * velocity_scale_factor_parameter.imu_rate;
    double estimated_buffer_number_max = velocity_scale_factor_parameter.estimated_maximum_interval * velocity_scale_factor_parameter.imu_rate;
    double enabled_data_ratio = velocity_scale_factor_parameter.gnss_rate / velocity_scale_factor_parameter.imu_rate * velocity_scale_factor_parameter.gnss_receiving_threshold;

  if(velocity_scale_factor_parameter.save_velocity_scale_factor)
  {
    if(velocity_scale_factor->status.enabled_status)
    {
      initial_velocity_scale_factor = velocity_scale_factor_status->velocity_scale_factor_last;
    }
  }

  if(velocity_scale_factor->status.enabled_status == true)
  {
    estimated_number_cur = estimated_buffer_number_max;
  }
  else
  {
    estimated_number_cur = estimated_buffer_number_min;
  }

  if (velocity_scale_factor_status->estimated_number < estimated_number_cur)
  {
    ++velocity_scale_factor_status->estimated_number;
  }
  else
  {
    velocity_scale_factor_status->estimated_number = estimated_number_cur;
  }

  gnss_status_buffer_length = std::distance(velocity_scale_factor_status->gnss_status_buffer.begin(), velocity_scale_factor_status->gnss_status_buffer.end());

  if (gnss_status_buffer_length > estimated_number_cur)
  {
    velocity_scale_factor_status->gnss_status_buffer.erase(velocity_scale_factor_status->gnss_status_buffer.begin());
    velocity_scale_factor_status->doppler_velocity_buffer.erase(velocity_scale_factor_status->doppler_velocity_buffer.begin());
    velocity_scale_factor_status->velocity_buffer.erase(velocity_scale_factor_status->velocity_buffer.begin());
  }

  std::vector<int> gnss_index;
  std::vector<int> velocity_index;
  std::vector<int> index;
  std::vector<double> velocity_scale_factor_buffer;

  if (velocity_scale_factor_status->estimated_number >= estimated_buffer_number_min &&
    velocity_scale_factor_status->gnss_status_buffer[velocity_scale_factor_status->estimated_number - 1] == true &&
    velocity_scale_factor_status->velocity_buffer[velocity_scale_factor_status->estimated_number - 1] >
    velocity_scale_factor_parameter.moving_judgment_threshold)
  {
    for (i = 0; i < velocity_scale_factor_status->estimated_number; i++)
    {
      if (velocity_scale_factor_status->gnss_status_buffer[i] == true)
      {
        gnss_index.push_back(i);
      }
      if (velocity_scale_factor_status->velocity_buffer[i] > velocity_scale_factor_parameter.moving_judgment_threshold)
      {
        velocity_index.push_back(i);
      }
    }

    set_intersection(gnss_index.begin(), gnss_index.end(), velocity_index.begin(), velocity_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > velocity_scale_factor_status->estimated_number * enabled_data_ratio)
    {
      for (i = 0; i < index_length; i++)
      {
        velocity_scale_factor_buffer.push_back(velocity_scale_factor_status->doppler_velocity_buffer[index[i]] /
          velocity_scale_factor_status->velocity_buffer[index[i]]);
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
    correction_velocity->twist.linear.x = velocity.twist.linear.x * velocity_scale_factor->scale_factor;
  }
  else
  {
    velocity_scale_factor->status.enabled_status = false;
    velocity_scale_factor->scale_factor = initial_velocity_scale_factor;
    correction_velocity->twist.linear.x = velocity.twist.linear.x * initial_velocity_scale_factor;
  }

  velocity_scale_factor_status->velocity_scale_factor_last = velocity_scale_factor->scale_factor;

}

void velocity_scale_factor_estimate(const rtklib_msgs::msg::RtklibNav rtklib_nav, const geometry_msgs::msg::TwistStamped velocity,
  const VelocityScaleFactorParameter velocity_scale_factor_parameter, VelocityScaleFactorStatus* velocity_scale_factor_status,
  geometry_msgs::msg::TwistStamped* correction_velocity, eagleye_msgs::msg::VelocityScaleFactor* velocity_scale_factor)
{

    double ecef_vel[3];
    double ecef_pos[3];
    double enu_vel[3];

    bool gnss_status,gnss_update;
    double doppler_velocity = 0.0;

    ecef_vel[0] = rtklib_nav.ecef_vel.x;
    ecef_vel[1] = rtklib_nav.ecef_vel.y;
    ecef_vel[2] = rtklib_nav.ecef_vel.z;
    ecef_pos[0] = rtklib_nav.ecef_pos.x;
    ecef_pos[1] = rtklib_nav.ecef_pos.y;
    ecef_pos[2] = rtklib_nav.ecef_pos.z;

    xyz2enu_vel(ecef_vel, ecef_pos, enu_vel);

    if (!std::isfinite(enu_vel[0])||!std::isfinite(enu_vel[1])||!std::isfinite(enu_vel[2]))
    {
      enu_vel[0] = 0;
      enu_vel[1] = 0;
      enu_vel[2] = 0;
      gnss_update = false;
    }
    else
    {
      gnss_update = true;
    }

    doppler_velocity = std::sqrt((enu_vel[0] * enu_vel[0]) + (enu_vel[1] * enu_vel[1]) + (enu_vel[2] * enu_vel[2]));

  if (velocity_scale_factor_status->tow_last == rtklib_nav.tow || rtklib_nav.tow == 0 || gnss_update == false)
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

  velocity_scale_factor_estimate_(velocity, velocity_scale_factor_parameter, velocity_scale_factor_status, correction_velocity, velocity_scale_factor);
}

void velocity_scale_factor_estimate(const nmea_msgs::msg::Gprmc nmea_rmc, const geometry_msgs::msg::TwistStamped velocity,
  const VelocityScaleFactorParameter velocity_scale_factor_parameter, VelocityScaleFactorStatus* velocity_scale_factor_status,
  geometry_msgs::msg::TwistStamped* correction_velocity, eagleye_msgs::msg::VelocityScaleFactor* velocity_scale_factor)
{
  bool gnss_status;
  double doppler_velocity = 0.0;

  if (velocity_scale_factor_status->rmc_time_last == nmea_rmc.utc_seconds || nmea_rmc.utc_seconds == 0)
  {
    gnss_status = false;
    doppler_velocity = 0;
    velocity_scale_factor_status->rmc_time_last = nmea_rmc.utc_seconds;
  }
  else
  {
    gnss_status = true;
    doppler_velocity = nmea_rmc.speed * knot2mps;
    velocity_scale_factor_status->rmc_time_last = nmea_rmc.utc_seconds;
  }

  velocity_scale_factor_status->gnss_status_buffer.push_back(gnss_status);
  velocity_scale_factor_status->doppler_velocity_buffer.push_back(doppler_velocity);
  velocity_scale_factor_status->velocity_buffer.push_back(velocity.twist.linear.x);

  velocity_scale_factor_estimate_(velocity, velocity_scale_factor_parameter, velocity_scale_factor_status, correction_velocity, velocity_scale_factor);
} 
