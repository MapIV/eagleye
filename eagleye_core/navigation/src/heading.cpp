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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void heading_estimate_(sensor_msgs::msg::Imu imu, geometry_msgs::msg::TwistStamped velocity, eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop,
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset, eagleye_msgs::msg::SlipAngle slip_angle, eagleye_msgs::msg::Heading heading_interpolate,
  HeadingParameter heading_parameter, HeadingStatus* heading_status, eagleye_msgs::msg::Heading* heading)
{
  int i,index_max;
  double yaw_rate = 0.0;
  double avg = 0.0,tmp_heading_angle;
  bool gnss_status,gnss_update;
  std::size_t index_length;
  std::size_t time_buffer_length;
  std::size_t inversion_up_index_length;
  std::size_t inversion_down_index_length;
  std::vector<double>::iterator max;

  double estimated_buffer_number_min = heading_parameter.estimated_minimum_interval * heading_parameter.imu_rate;
  double estimated_buffer_number_max = heading_parameter.estimated_maximum_interval * heading_parameter.imu_rate;
  double enabled_data_ratio = heading_parameter.gnss_rate / heading_parameter.imu_rate * heading_parameter.gnss_receiving_threshold;
  double remain_data_ratio = enabled_data_ratio * heading_parameter.outlier_ratio_threshold;

  rclcpp::Time ros_clock(imu.header.stamp);
  auto imu_time = ros_clock.seconds();

  if (heading_status->estimated_number  < estimated_buffer_number_max)
  {
    ++heading_status->estimated_number ;
  }
  else
  {
    heading_status->estimated_number  = estimated_buffer_number_max;
  }

  yaw_rate = imu.angular_velocity.z;

  // data buffer generate
  heading_status->time_buffer .push_back(imu_time);
  heading_status->yaw_rate_buffer .push_back(yaw_rate);
  heading_status->correction_velocity_buffer .push_back(velocity.twist.linear.x);
  heading_status->yaw_rate_offset_stop_buffer .push_back(yaw_rate_offset_stop.yaw_rate_offset);
  heading_status->yaw_rate_offset_buffer .push_back(yaw_rate_offset.yaw_rate_offset);
  heading_status->slip_angle_buffer .push_back(slip_angle.slip_angle);

  time_buffer_length = std::distance(heading_status->time_buffer .begin(), heading_status->time_buffer .end());

  if (time_buffer_length > estimated_buffer_number_max)
  {
    heading_status->time_buffer .erase(heading_status->time_buffer .begin());
    heading_status->heading_angle_buffer .erase(heading_status->heading_angle_buffer .begin());
    heading_status->yaw_rate_buffer .erase(heading_status->yaw_rate_buffer .begin());
    heading_status->correction_velocity_buffer .erase(heading_status->correction_velocity_buffer .begin());
    heading_status->yaw_rate_offset_stop_buffer .erase(heading_status->yaw_rate_offset_stop_buffer .begin());
    heading_status->yaw_rate_offset_buffer .erase(heading_status->yaw_rate_offset_buffer .begin());
    heading_status->slip_angle_buffer .erase(heading_status->slip_angle_buffer .begin());
    heading_status->gnss_status_buffer .erase(heading_status->gnss_status_buffer .begin());
  }

  std::vector<int> gnss_index;
  std::vector<int> velocity_index;
  std::vector<int> index;

  if (heading_status->estimated_number  > estimated_buffer_number_min &&
    heading_status->gnss_status_buffer [heading_status->estimated_number -1] == true &&
    heading_status->correction_velocity_buffer [heading_status->estimated_number -1] > heading_parameter.moving_judgment_threshold &&
    fabsf(heading_status->yaw_rate_buffer [heading_status->estimated_number -1]) < heading_parameter.curve_judgment_threshold)
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
      if (heading_status->correction_velocity_buffer [i] > heading_parameter.moving_judgment_threshold)
      {
        velocity_index.push_back(i);
      }
    }

    set_intersection(gnss_index.begin(), gnss_index.end(), velocity_index.begin(), velocity_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > heading_status->estimated_number  * enabled_data_ratio)
    {
      std::vector<double> provisional_heading_angle_buffer(heading_status->estimated_number , 0);

      for (i = 0; i < heading_status->estimated_number ; i++)
      {
        if (i > 0)
        {
          if (std::abs(heading_status->correction_velocity_buffer [heading_status->estimated_number -1]) > heading_parameter.moving_judgment_threshold)
          {
            provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] +
              ((heading_status->yaw_rate_buffer [i] + heading_status->yaw_rate_offset_buffer [i]) *
              (heading_status->time_buffer [i] - heading_status->time_buffer [i-1]));
          }
          else
          {
            provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] +
              ((heading_status->yaw_rate_buffer [i] + heading_status->yaw_rate_offset_stop_buffer [i]) *
              (heading_status->time_buffer [i] - heading_status->time_buffer [i-1]));
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
        base_heading_angle_buffer.push_back(heading_interpolate.heading_angle - provisional_heading_angle_buffer[index[index_length-1]] +
          provisional_heading_angle_buffer[i]);
      }

      for (i = 0; i < index_length; i++)
      {
        ref_cnt = (base_heading_angle_buffer[index[i]] - std::fmod(base_heading_angle_buffer[index[i]],2*M_PI))/(2*M_PI);
        if(base_heading_angle_buffer[index[i]] < 0) ref_cnt = ref_cnt -1;
        heading_angle_buffer2[index[i]] = heading_status->heading_angle_buffer [index[i]] + ref_cnt * 2*M_PI;
      }

      while (1)
      {
        index_length = std::distance(index.begin(), index.end());

        base_heading_angle_buffer.clear();
        for (i = 0; i < heading_status->estimated_number ; i++)
        {
          base_heading_angle_buffer.push_back(heading_angle_buffer2[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] +
            provisional_heading_angle_buffer[i]);
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
          base_heading_angle_buffer2.push_back(tmp_heading_angle - provisional_heading_angle_buffer[index[index_length-1]] +
            provisional_heading_angle_buffer[i]);
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

        if (index_length < heading_status->estimated_number  * remain_data_ratio)
        {
          break;
        }
      }

      if (index_length == 0 || index_length > heading_status->estimated_number  * remain_data_ratio)
      {
        if (index[index_length-1] == heading_status->estimated_number -1)
        {
          heading->heading_angle = tmp_heading_angle;
        }
        else
        {
          heading->heading_angle = tmp_heading_angle + (provisional_heading_angle_buffer[heading_status->estimated_number -1] -
            provisional_heading_angle_buffer[index[index_length-1]]);
        }
        heading->status.estimate_status = true;

        double heading_STD = heading_parameter.init_STD;
        heading->variance = heading_STD*heading_STD;
      }
    }
  }
}

void heading_estimate(rtklib_msgs::msg::RtklibNav rtklib_nav,sensor_msgs::msg::Imu imu, geometry_msgs::msg::TwistStamped velocity,
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop,eagleye_msgs::msg::YawrateOffset yaw_rate_offset,eagleye_msgs::msg::SlipAngle slip_angle,
  eagleye_msgs::msg::Heading heading_interpolate,HeadingParameter heading_parameter, HeadingStatus* heading_status,eagleye_msgs::msg::Heading* heading)
{
  double ecef_vel[3];
  double ecef_pos[3];
  double enu_vel[3];

  double doppler_heading_angle = 0.0;
  bool gnss_status,gnss_update;

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
  else{
    gnss_update = true;
  }

  doppler_heading_angle = std::atan2(enu_vel[0], enu_vel[1]);

  if(doppler_heading_angle<0){
    doppler_heading_angle = doppler_heading_angle + 2*M_PI;
  }

  if (heading_status->tow_last  == rtklib_nav.tow || rtklib_nav.tow == 0 || gnss_update == false)
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

  heading_status->heading_angle_buffer .push_back(doppler_heading_angle);
  heading_status->gnss_status_buffer .push_back(gnss_status);

  heading_estimate_(imu, velocity, yaw_rate_offset_stop, yaw_rate_offset, slip_angle, heading_interpolate, heading_parameter, heading_status, heading);
}

void heading_estimate(const nmea_msgs::msg::Gprmc nmea_rmc,sensor_msgs::msg::Imu imu, geometry_msgs::msg::TwistStamped velocity,
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop,eagleye_msgs::msg::YawrateOffset yaw_rate_offset,eagleye_msgs::msg::SlipAngle slip_angle,
  eagleye_msgs::msg::Heading heading_interpolate,HeadingParameter heading_parameter, HeadingStatus* heading_status,eagleye_msgs::msg::Heading* heading)
{
  bool gnss_status;
  double doppler_heading_angle = 0.0;

  if (heading_status->rmc_time_last == nmea_rmc.utc_seconds || nmea_rmc.utc_seconds == 0 || nmea_rmc.track == 0)
  {
    gnss_status = false;
    doppler_heading_angle = 0;
    heading_status->rmc_time_last = nmea_rmc.utc_seconds;
  }
  else
  {
    gnss_status = true;
    doppler_heading_angle = nmea_rmc.track * M_PI/180;
    heading_status->rmc_time_last = nmea_rmc.utc_seconds;
  }

  heading_status->heading_angle_buffer.push_back(doppler_heading_angle);
  heading_status->gnss_status_buffer.push_back(gnss_status);

  heading_estimate_(imu, velocity, yaw_rate_offset_stop, yaw_rate_offset, slip_angle, heading_interpolate, heading_parameter, heading_status, heading);
}

void heading_estimate(const eagleye_msgs::msg::Heading multi_antenna_heading,sensor_msgs::msg::Imu imu,geometry_msgs::msg::TwistStamped velocity,
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop,eagleye_msgs::msg::YawrateOffset yaw_rate_offset,eagleye_msgs::msg::SlipAngle slip_angle,
  eagleye_msgs::msg::Heading heading_interpolate,HeadingParameter heading_parameter, HeadingStatus* heading_status,eagleye_msgs::msg::Heading* heading)
{
  bool gnss_status;
  double heading_angle = 0.0;

  rclcpp::Time multi_anttena_clock(multi_antenna_heading.header.stamp);
  double multi_anttena_time = multi_anttena_clock.seconds();
  if (heading_status->ros_time_last ==  multi_anttena_time || multi_anttena_time == 0)
  {
    gnss_status = false;
    heading_angle = 0;
    heading_status->ros_time_last = multi_anttena_time;
  }
  else
  {
    gnss_status = true;
    heading_angle = multi_antenna_heading.heading_angle;
    heading_status->ros_time_last = multi_anttena_time;
  }

  heading_status->heading_angle_buffer .push_back(heading_angle);
  heading_status->gnss_status_buffer .push_back(gnss_status);

  heading_estimate_(imu,velocity,yaw_rate_offset_stop,yaw_rate_offset,slip_angle,heading_interpolate,heading_parameter,heading_status,heading);

  if(!heading->status.estimate_status) heading->heading_angle = heading_angle;
  if(gnss_status) heading->status.estimate_status = true;

}