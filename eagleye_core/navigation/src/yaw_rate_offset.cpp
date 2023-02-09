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
 * yaw_rate_offset.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void yaw_rate_offset_estimate(const geometry_msgs::msg::TwistStamped velocity, const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop,
  const eagleye_msgs::msg::Heading heading_interpolate,const sensor_msgs::msg::Imu imu, const YawrateOffsetParameter yaw_rate_offset_parameter,
  YawrateOffsetStatus* yaw_rate_offset_status, eagleye_msgs::msg::YawrateOffset* yaw_rate_offset)
{
  int i;
  double yaw_rate = 0.0;
  double sum_xy, sum_x, sum_y, sum_x2;
  bool estimated_condition_status;

  std::size_t index_length;
  std::size_t time_buffer_length;
  std::size_t inversion_up_index_length;
  std::size_t inversion_down_index_length;
  double estimated_number_cur;

  double estimated_buffer_number_min = yaw_rate_offset_parameter.estimated_minimum_interval * yaw_rate_offset_parameter.imu_rate;
  double estimated_buffer_number_max = yaw_rate_offset_parameter.estimated_maximum_interval * yaw_rate_offset_parameter.imu_rate;
  double enabled_data_ratio = yaw_rate_offset_parameter.gnss_rate / yaw_rate_offset_parameter.imu_rate * yaw_rate_offset_parameter.gnss_receiving_threshold;

  rclcpp::Time ros_clock(imu.header.stamp);
  auto imu_time = ros_clock.seconds();

  if(yaw_rate_offset->status.enabled_status == true)
  {
    estimated_number_cur = estimated_buffer_number_max;
  }
  else
  {
    estimated_number_cur = estimated_buffer_number_min;
  }

  if (yaw_rate_offset_status->estimated_number < estimated_number_cur)
  {
    ++yaw_rate_offset_status->estimated_number;
  }
  else
  {
    yaw_rate_offset_status->estimated_number = estimated_number_cur;
  }

  yaw_rate = imu.angular_velocity.z;

  // data buffer generate
  yaw_rate_offset_status->time_buffer.push_back(imu_time);
  yaw_rate_offset_status->yaw_rate_buffer.push_back(yaw_rate);
  yaw_rate_offset_status->heading_angle_buffer.push_back(heading_interpolate.heading_angle);
  yaw_rate_offset_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);
  yaw_rate_offset_status->heading_estimate_status_buffer.push_back(heading_interpolate.status.estimate_status);
  yaw_rate_offset_status->yaw_rate_offset_stop_buffer.push_back(yaw_rate_offset_stop.yaw_rate_offset);

  time_buffer_length = std::distance(yaw_rate_offset_status->time_buffer.begin(), yaw_rate_offset_status->time_buffer.end());

  if (time_buffer_length > estimated_number_cur)
  {
    yaw_rate_offset_status->time_buffer.erase(yaw_rate_offset_status->time_buffer.begin());
    yaw_rate_offset_status->yaw_rate_buffer.erase(yaw_rate_offset_status->yaw_rate_buffer.begin());
    yaw_rate_offset_status->heading_angle_buffer.erase(yaw_rate_offset_status->heading_angle_buffer.begin());
    yaw_rate_offset_status->correction_velocity_buffer.erase(yaw_rate_offset_status->correction_velocity_buffer.begin());
    yaw_rate_offset_status->heading_estimate_status_buffer.erase(yaw_rate_offset_status->heading_estimate_status_buffer.begin());
    yaw_rate_offset_status->yaw_rate_offset_stop_buffer.erase(yaw_rate_offset_status->yaw_rate_offset_stop_buffer.begin());
  }

  if (yaw_rate_offset_status->estimated_preparation_conditions == 0 &&
    yaw_rate_offset_status->heading_estimate_status_buffer[yaw_rate_offset_status->estimated_number - 1] == true)
  {
    yaw_rate_offset_status->estimated_preparation_conditions = 1;
  }
  else if (yaw_rate_offset_status->estimated_preparation_conditions == 1)
  {
    if (yaw_rate_offset_status->heading_estimate_status_count < estimated_buffer_number_min)
    {
      ++yaw_rate_offset_status->heading_estimate_status_count;
    }
    else if (yaw_rate_offset_status->heading_estimate_status_count == estimated_buffer_number_min)
    {
      yaw_rate_offset_status->estimated_preparation_conditions = 2;
    }
  }

  if (yaw_rate_offset_status->estimated_preparation_conditions == 2 && yaw_rate_offset_status->correction_velocity_buffer[yaw_rate_offset_status->estimated_number-1] >
    yaw_rate_offset_parameter.moving_judgment_threshold && yaw_rate_offset_status->heading_estimate_status_buffer[yaw_rate_offset_status->estimated_number-1] == true)
  {
    estimated_condition_status = true;
  }
  else
  {
    estimated_condition_status = false;
  }

  std::vector<int> velocity_index;
  std::vector<int> heading_estimate_status_index;
  std::vector<int> index;

  if (estimated_condition_status == true)
  {
    for (i = 0; i < yaw_rate_offset_status->estimated_number; i++)
    {
      if (yaw_rate_offset_status->correction_velocity_buffer[i] > yaw_rate_offset_parameter.moving_judgment_threshold)
      {
        velocity_index.push_back(i);
      }
      if (yaw_rate_offset_status->heading_estimate_status_buffer[i] == true)
      {
        heading_estimate_status_index.push_back(i);
      }
    }

    set_intersection(velocity_index.begin(), velocity_index.end(), heading_estimate_status_index.begin(), heading_estimate_status_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > yaw_rate_offset_status->estimated_number * enabled_data_ratio)
    {
      std::vector<double> provisional_heading_angle_buffer(yaw_rate_offset_status->estimated_number, 0);

      for (i = 0; i < yaw_rate_offset_status->estimated_number; i++)
      {
        if (i > 0)
        {
          provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] +
            yaw_rate_offset_status->yaw_rate_buffer[i] * (yaw_rate_offset_status->time_buffer[i] - yaw_rate_offset_status->time_buffer[i-1]);
        }
      }

      std::vector<double> base_heading_angle_buffer;
      std::vector<double> diff_buffer;
      std::vector<double> time_buffer2;
      std::vector<double> inversion_up_index;
      std::vector<double> inversion_down_index;

      index_length = std::distance(index.begin(), index.end());

      //base_heading_angle_buffer.clear();
      for (i = 0; i < yaw_rate_offset_status->estimated_number; i++)
      {
        base_heading_angle_buffer.push_back(yaw_rate_offset_status->heading_angle_buffer[index[index_length-1]] -
          provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
      }

      //diff_buffer.clear();
      for (i = 0; i < index_length; i++)
      {
        // diff_buffer.push_back(base_heading_angle_buffer[index[i]] - heading_angle_buffer[index[i]]);
        diff_buffer.push_back(yaw_rate_offset_status->heading_angle_buffer[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] +
          provisional_heading_angle_buffer[index[i]] - yaw_rate_offset_status->heading_angle_buffer[index[i]]);
      }

      time_buffer2.clear();
      for (i = 0; i < index_length; i++)
      {
        time_buffer2.push_back(yaw_rate_offset_status->time_buffer[index[i]] - yaw_rate_offset_status->time_buffer[index[0]]);
      }

      // Least-square
      sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
      for (i = 0; i < index_length ; i++)
      {
        sum_xy += time_buffer2[i] * diff_buffer[i];
        sum_x += time_buffer2[i];
        sum_y += diff_buffer[i];
        sum_x2 += std::pow(time_buffer2[i], 2);
      }
      yaw_rate_offset->yaw_rate_offset = -1 * (index_length * sum_xy - sum_x * sum_y) / (index_length * sum_x2 - pow(sum_x, 2));
      yaw_rate_offset->status.enabled_status = true;
      yaw_rate_offset->status.estimate_status = true;
    }
  }

  if (yaw_rate_offset->status.enabled_status == false)
  {
    yaw_rate_offset->yaw_rate_offset = yaw_rate_offset_stop.yaw_rate_offset;
  }

  if (std::fabs(yaw_rate_offset->yaw_rate_offset - yaw_rate_offset_stop.yaw_rate_offset) > yaw_rate_offset_parameter.outlier_threshold)
  {
    yaw_rate_offset->yaw_rate_offset = yaw_rate_offset_stop.yaw_rate_offset;
  }

}
