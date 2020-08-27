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
 * heading.cpp
 * Author MapIV Sekino
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void heading_estimate(rtklib_msgs::RtklibNav rtklib_nav,sensor_msgs::Imu imu,eagleye_msgs::VelocityScaleFactor velocity_scale_factor,eagleye_msgs::YawrateOffset yawrate_offset_stop,eagleye_msgs::YawrateOffset yawrate_offset,eagleye_msgs::SlipAngle slip_angle,eagleye_msgs::Heading heading_interpolate,HeadingParameter heading_parameter, HeadingStatus* heading_status,eagleye_msgs::Heading* heading)
{

  double ecef_vel[3];
  double ecef_pos[3];
  double enu_vel[3];

  int i,index_max;
  double yawrate = 0.0 , doppler_heading_angle = 0.0;
  double avg = 0.0,tmp_heading_angle;
  bool gnss_status;
  std::size_t index_length;
  std::size_t time_buffer_length;
  std::size_t inversion_up_index_length;
  std::size_t inversion_down_index_length;
  std::vector<double>::iterator max;

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

  if (heading_status->estimated_number  < heading_parameter.estimated_number_max)
  {
    ++heading_status->estimated_number ;
  }
  else
  {
    heading_status->estimated_number  = heading_parameter.estimated_number_max;
  }

  if (heading_parameter.reverse_imu == false)
  {
    yawrate = imu.angular_velocity.z;
  }
  else if (heading_parameter.reverse_imu == true)
  {
    yawrate = -1 * imu.angular_velocity.z;
  }

  if (heading_status->tow_last  == rtklib_nav.tow)
  {
    gnss_status = false;
    doppler_heading_angle = 0;
    heading_status->tow_last  = rtklib_nav.tow;
  }
  else
  {
    gnss_status = true;
    doppler_heading_angle = doppler_heading_angle;
    heading_status->tow_last  = rtklib_nav.tow;
  }

  // data buffer generate
  heading_status->time_buffer .push_back(imu.header.stamp.toSec());
  heading_status->heading_angle_buffer .push_back(doppler_heading_angle);
  heading_status->yawrate_buffer .push_back(yawrate);
  heading_status->correction_velocity_buffer .push_back(velocity_scale_factor.correction_velocity.linear.x);
  heading_status->yawrate_offset_stop_buffer .push_back(yawrate_offset_stop.yawrate_offset);
  heading_status->yawrate_offset_buffer .push_back(yawrate_offset.yawrate_offset);
  heading_status->slip_angle_buffer .push_back(slip_angle.slip_angle);
  heading_status->gnss_status_buffer .push_back(gnss_status);

  time_buffer_length = std::distance(heading_status->time_buffer .begin(), heading_status->time_buffer .end());

  if (time_buffer_length > heading_parameter.estimated_number_max)
  {
    heading_status->time_buffer .erase(heading_status->time_buffer .begin());
    heading_status->heading_angle_buffer .erase(heading_status->heading_angle_buffer .begin());
    heading_status->yawrate_buffer .erase(heading_status->yawrate_buffer .begin());
    heading_status->correction_velocity_buffer .erase(heading_status->correction_velocity_buffer .begin());
    heading_status->yawrate_offset_stop_buffer .erase(heading_status->yawrate_offset_stop_buffer .begin());
    heading_status->yawrate_offset_buffer .erase(heading_status->yawrate_offset_buffer .begin());
    heading_status->slip_angle_buffer .erase(heading_status->slip_angle_buffer .begin());
    heading_status->gnss_status_buffer .erase(heading_status->gnss_status_buffer .begin());
  }

  std::vector<int> gnss_index;
  std::vector<int> velocity_index;
  std::vector<int> index;

  if (heading_status->estimated_number  > heading_parameter.estimated_number_min && heading_status->gnss_status_buffer [heading_status->estimated_number -1] == true && heading_status->correction_velocity_buffer [heading_status->estimated_number -1] > heading_parameter.estimated_velocity_threshold && fabsf(heading_status->yawrate_buffer [heading_status->estimated_number -1]) < heading_parameter.estimated_yawrate_threshold)
  {
    heading->status.enabled_status = true;
  }
  else
  {
    heading->status.enabled_status = false;
  }

  if (heading->status.enabled_status == true)
  {
    for (i = 0; i < heading_status->estimated_number ; i++)
    {
      if (heading_status->gnss_status_buffer [i] == true)
      {
        gnss_index.push_back(i);
      }
      if (heading_status->correction_velocity_buffer [i] > heading_parameter.estimated_velocity_threshold)
      {
        velocity_index.push_back(i);
      }
    }

    set_intersection(gnss_index.begin(), gnss_index.end(), velocity_index.begin(), velocity_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > heading_status->estimated_number  * heading_parameter.estimated_gnss_coefficient)
    {
      std::vector<double> provisional_heading_angle_buffer(heading_status->estimated_number , 0);

      for (i = 0; i < heading_status->estimated_number ; i++)
      {
        if (i > 0)
        {
          if (std::abs(heading_status->correction_velocity_buffer [heading_status->estimated_number -1]) > heading_parameter.stop_judgment_velocity_threshold)
          {
            provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] + ((heading_status->yawrate_buffer [i] + heading_status->yawrate_offset_buffer [i]) * (heading_status->time_buffer [i] - heading_status->time_buffer [i-1]));
          }
          else
          {
            provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] + ((heading_status->yawrate_buffer [i] + heading_status->yawrate_offset_stop_buffer [i]) * (heading_status->time_buffer [i] - heading_status->time_buffer [i-1]));
          }
        }
      }

      std::vector<double> base_heading_angle_buffer;
      std::vector<double> base_heading_angle_buffer2;
      std::vector<double> diff_buffer;
      std::vector<double> inversion_up_index;
      std::vector<double> inversion_down_index;

     if(heading_interpolate.status.enabled_status == false)
     {
       heading_interpolate.heading_angle = heading_status->heading_angle_buffer [index[index_length-1]];
     }

      int ref_cnt;
      std::vector<double> heading_angle_buffer2;

      copy(heading_status->heading_angle_buffer .begin(), heading_status->heading_angle_buffer .end(), back_inserter(heading_angle_buffer2) );

      for (i = 0; i < heading_status->estimated_number ; i++)
      {
        base_heading_angle_buffer.push_back(heading_interpolate.heading_angle - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
      }

      for (i = 0; i < index_length; i++)
      {
        ref_cnt = (base_heading_angle_buffer[index[i]] - fmod(base_heading_angle_buffer[index[i]],2*M_PI))/(2*M_PI);
        if(base_heading_angle_buffer[index[i]] < 0) ref_cnt = ref_cnt -1;
        heading_angle_buffer2[index[i]] = heading_status->heading_angle_buffer [index[i]] + ref_cnt * 2*M_PI;
      }

      while (1)
      {
        index_length = std::distance(index.begin(), index.end());

        base_heading_angle_buffer.clear();
        for (i = 0; i < heading_status->estimated_number ; i++)
        {
          base_heading_angle_buffer.push_back(heading_angle_buffer2[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
        }

        diff_buffer.clear();
        for (i = 0; i < index_length; i++)
        {
          diff_buffer.push_back(base_heading_angle_buffer[index[i]] - heading_angle_buffer2[index[i]]);
        }

        avg = std::accumulate(diff_buffer.begin(), diff_buffer.end(), 0.0) / index_length;
        tmp_heading_angle = heading_angle_buffer2[index[index_length-1]] - avg;

        base_heading_angle_buffer2.clear();
        for (i = 0; i < heading_status->estimated_number ; i++)
        {
          base_heading_angle_buffer2.push_back(tmp_heading_angle - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
        }

        diff_buffer.clear();
        for (i = 0; i < index_length; i++)
        {
          diff_buffer.push_back(fabsf(base_heading_angle_buffer2[index[i]] - heading_angle_buffer2[index[i]]));
        }

        max = std::max_element(diff_buffer.begin(), diff_buffer.end());
        index_max = std::distance(diff_buffer.begin(), max);

        if (diff_buffer[index_max] > heading_parameter.outlier_threshold)
        {
          index.erase(index.begin() + index_max);
        }
        else
        {
          break;
        }

        index_length = std::distance(index.begin(), index.end());

        if (index_length < heading_status->estimated_number  * heading_parameter.estimated_heading_coefficient)
        {
          break;
        }
      }

      if (index_length == 0 || index_length > heading_status->estimated_number  * heading_parameter.estimated_heading_coefficient)
      {
        if (index[index_length-1] == heading_status->estimated_number -1)
        {
          heading->heading_angle = tmp_heading_angle;
        }
        else
        {
          heading->heading_angle = tmp_heading_angle + (provisional_heading_angle_buffer[heading_status->estimated_number -1] - provisional_heading_angle_buffer[index[index_length-1]]);
        }
        heading->status.estimate_status = true;
      }
    }
  }
}
