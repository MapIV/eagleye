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
 * slip_coefficient.cpp
 * Author MapIV Takanose
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void slip_coefficient_estimate(sensor_msgs::Imu imu,rtklib_msgs::RtklibNav rtklib_nav,eagleye_msgs::VelocityScaleFactor velocity_scale_factor,eagleye_msgs::YawrateOffset yawrate_offset_stop,eagleye_msgs::YawrateOffset yawrate_offset_2nd,eagleye_msgs::Heading heading_interpolate_3rd,SlipCoefficientParameter slip_coefficient_parameter,SlipCoefficientStatus* slip_coefficient_status,double* estimate_coefficient)
{

  int i;
  double doppler_heading_angle;
  double doppler_slip;
  double rear_slip;
  double yawrate;
  double acceleration_y;
  double sum_xy, sum_x, sum_y, sum_x2;
  double ecef_vel[3];
  double ecef_pos[3];
  double enu_vel[3];
  std::size_t acceleration_y_buffer_length;

  ecef_vel[0] = rtklib_nav.ecef_vel.x;
  ecef_vel[1] = rtklib_nav.ecef_vel.y;
  ecef_vel[2] = rtklib_nav.ecef_vel.z;
  ecef_pos[0] = rtklib_nav.ecef_pos.x;
  ecef_pos[1] = rtklib_nav.ecef_pos.y;
  ecef_pos[2] = rtklib_nav.ecef_pos.z;

  xyz2enu_vel(ecef_vel, ecef_pos, enu_vel);
  doppler_heading_angle = atan2(enu_vel[0], enu_vel[1]);

  if(doppler_heading_angle<0){
    doppler_heading_angle = doppler_heading_angle + 2*M_PI;
  }


  if (slip_coefficient_parameter.reverse_imu == false)
  {
    yawrate = imu.angular_velocity.z;
  }
  else if (slip_coefficient_parameter.reverse_imu == true)
  {
    yawrate = -1 * imu.angular_velocity.z;
  }

  if (std::abs(velocity_scale_factor.correction_velocity.linear.x) > slip_coefficient_parameter.stop_judgment_velocity_threshold)
  {
    yawrate = yawrate + yawrate_offset_2nd.yawrate_offset;
  }
  else
  {
    yawrate = yawrate + yawrate_offset_stop.yawrate_offset;
  }

  acceleration_y = velocity_scale_factor.correction_velocity.linear.x * yawrate;

  if (heading_interpolate_3rd.status.estimate_status == true)
  {
    if ((velocity_scale_factor.correction_velocity.linear.x > slip_coefficient_parameter.estimated_velocity_threshold) && (fabs(yawrate) > slip_coefficient_parameter.estimated_yawrate_threshold))
    {
      double imu_heading;

      if(fmod(heading_interpolate_3rd.heading_angle,2*M_PI)<0){
        imu_heading = fmod(heading_interpolate_3rd.heading_angle,2*M_PI) + 2*M_PI;
      }else{
        imu_heading = fmod(heading_interpolate_3rd.heading_angle,2*M_PI);
      }

      doppler_slip = (imu_heading - doppler_heading_angle);
      // ROS_INFO("IMU = %lf  GNSS = %lf",imu_heading,doppler_heading_angle);
      // ROS_INFO("doppler_slip = %lf",doppler_slip);

      rear_slip = doppler_slip + slip_coefficient_parameter.lever_arm*yawrate/velocity_scale_factor.correction_velocity.linear.x;
      // ROS_WARN("rear_slip = %lf",rear_slip);

      if(fabs(rear_slip)<(2*M_PI/180))
      {
        slip_coefficient_status->acceleration_y_buffer.push_back(acceleration_y);
        slip_coefficient_status->doppler_slip_buffer.push_back(rear_slip);

        acceleration_y_buffer_length = std::distance(slip_coefficient_status->acceleration_y_buffer.begin(), slip_coefficient_status->acceleration_y_buffer.end());

        if (acceleration_y_buffer_length > slip_coefficient_parameter.estimated_number_max)
        {
          slip_coefficient_status->acceleration_y_buffer.erase(slip_coefficient_status->acceleration_y_buffer.begin());
          slip_coefficient_status->doppler_slip_buffer.erase(slip_coefficient_status->doppler_slip_buffer.begin());
        }

        if(slip_coefficient_status->heading_estimate_status_count < slip_coefficient_parameter.estimated_number_max)
        {
          ++slip_coefficient_status->heading_estimate_status_count;
        }
        else
        {
          slip_coefficient_status->heading_estimate_status_count = slip_coefficient_parameter.estimated_number_max;
        }

        // ROS_INFO("slip_coefficient_status->heading_estimate_status_count = %d",slip_coefficient_status->heading_estimate_status_count);

        if(slip_coefficient_status->heading_estimate_status_count > slip_coefficient_parameter.estimated_number_min)
          {
            double sum_xy_avg,sum_x_square = 0.0;
            acceleration_y_buffer_length = std::distance(slip_coefficient_status->acceleration_y_buffer.begin(), slip_coefficient_status->acceleration_y_buffer.end());
            // ROS_INFO("slip_coefficient_status->acceleration_y_buffer_length = %lu",acceleration_y_buffer_length);

            // Least-square
            sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
            for (i = 0; i < acceleration_y_buffer_length ; i++)
            {
              sum_xy += slip_coefficient_status->acceleration_y_buffer[i] * slip_coefficient_status->doppler_slip_buffer[i];
              sum_x += slip_coefficient_status->acceleration_y_buffer[i];
              sum_y += slip_coefficient_status->doppler_slip_buffer[i];
              sum_x2 += pow(slip_coefficient_status->acceleration_y_buffer[i], 2);
            }
            *estimate_coefficient = (slip_coefficient_status->heading_estimate_status_count * sum_xy - sum_x * sum_y) / (slip_coefficient_status->heading_estimate_status_count * sum_x2 - pow(sum_x, 2));
            // ROS_ERROR("estimate_coefficient = %lf",estimate_coefficient);
          }
        }
      }
    }
}
