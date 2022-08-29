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
 * yawrate_offset.cpp
 * Author MapIV Sekino
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void yawrate_offset_estimate(const geometry_msgs::TwistStamped velocity, const eagleye_msgs::YawrateOffset yawrate_offset_stop,
  const eagleye_msgs::Heading heading_interpolate,const sensor_msgs::Imu imu, const YawrateOffsetParameter yawrate_offset_parameter,
  YawrateOffsetStatus* yawrate_offset_status, eagleye_msgs::YawrateOffset* yawrate_offset)
{
  double yawrate = imu.angular_velocity.z;
  std::size_t index_length;
  double estimated_number_cur;

  if(yawrate_offset->status.enabled_status == true)
  {
    estimated_number_cur = yawrate_offset_parameter.estimated_number_max;
  }
  else
  {
    estimated_number_cur = yawrate_offset_parameter.estimated_number_min;
  }

  if (yawrate_offset_status->estimated_number < estimated_number_cur)
  {
    ++yawrate_offset_status->estimated_number;
  }
  else
  {
    yawrate_offset_status->estimated_number = estimated_number_cur;
  }

  // data buffer generate
  yawrate_offset_status->time_buffer.push_back(imu.header.stamp.toSec());
  yawrate_offset_status->yawrate_buffer.push_back(yawrate);
  yawrate_offset_status->heading_angle_buffer.push_back(heading_interpolate.heading_angle);
  yawrate_offset_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);
  yawrate_offset_status->heading_estimate_status_buffer.push_back(heading_interpolate.status.estimate_status);
  yawrate_offset_status->yawrate_offset_stop_buffer.push_back(yawrate_offset_stop.yawrate_offset);

  std::size_t time_buffer_length = std::distance(yawrate_offset_status->time_buffer.begin(), yawrate_offset_status->time_buffer.end());

  if (time_buffer_length >estimated_number_cur)
  {
    yawrate_offset_status->time_buffer.erase(yawrate_offset_status->time_buffer.begin());
    yawrate_offset_status->yawrate_buffer.erase(yawrate_offset_status->yawrate_buffer.begin());
    yawrate_offset_status->heading_angle_buffer.erase(yawrate_offset_status->heading_angle_buffer.begin());
    yawrate_offset_status->correction_velocity_buffer.erase(yawrate_offset_status->correction_velocity_buffer.begin());
    yawrate_offset_status->heading_estimate_status_buffer.erase(yawrate_offset_status->heading_estimate_status_buffer.begin());
    yawrate_offset_status->yawrate_offset_stop_buffer.erase(yawrate_offset_status->yawrate_offset_stop_buffer.begin());
  }

  if (yawrate_offset_status->estimated_preparation_conditions == 0 &&
    yawrate_offset_status->heading_estimate_status_buffer[yawrate_offset_status->estimated_number - 1])
  {
    yawrate_offset_status->estimated_preparation_conditions = 1;
  }
  else if (yawrate_offset_status->estimated_preparation_conditions == 1)
  {
    if (yawrate_offset_status->heading_estimate_status_count < yawrate_offset_parameter.estimated_number_min)
    {
      ++yawrate_offset_status->heading_estimate_status_count;
    }
    else if (yawrate_offset_status->heading_estimate_status_count == yawrate_offset_parameter.estimated_number_min)
    {
      yawrate_offset_status->estimated_preparation_conditions = 2;
    }
  }

  bool estimated_condition_status;
  if (yawrate_offset_status->estimated_preparation_conditions == 2 &&
    yawrate_offset_status->correction_velocity_buffer[yawrate_offset_status->estimated_number-1] > yawrate_offset_parameter.estimated_velocity_threshold &&
    yawrate_offset_status->heading_estimate_status_buffer[yawrate_offset_status->estimated_number-1])
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

  if (estimated_condition_status)
  {
    for (int i = 0; i < yawrate_offset_status->estimated_number; i++)
    {
      if (yawrate_offset_status->correction_velocity_buffer[i] > yawrate_offset_parameter.estimated_velocity_threshold)
      {
        velocity_index.push_back(i);
      }
      if (yawrate_offset_status->heading_estimate_status_buffer[i])
      {
        heading_estimate_status_index.push_back(i);
      }
    }

    set_intersection(velocity_index.begin(), velocity_index.end(), heading_estimate_status_index.begin(), heading_estimate_status_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > yawrate_offset_status->estimated_number * yawrate_offset_parameter.estimated_coefficient)
    {
      std::vector<double> provisional_heading_angle_buffer(yawrate_offset_status->estimated_number, 0);

      for (int i = 0; i < yawrate_offset_status->estimated_number; i++)
      {
        if (i > 0)
        {
          provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] + yawrate_offset_status->yawrate_buffer[i] *
            (yawrate_offset_status->time_buffer[i] - yawrate_offset_status->time_buffer[i-1]);
        }
      }

      std::vector<double> diff_buffer;
      std::vector<double> time_buffer2;
      std::vector<double> inversion_up_index;
      std::vector<double> inversion_down_index;

      index_length = std::distance(index.begin(), index.end());

      for (int i = 0; i < index_length; i++)
      {
        diff_buffer.push_back(yawrate_offset_status->heading_angle_buffer[index[index_length-1]] -
          provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[index[i]] -
          yawrate_offset_status->heading_angle_buffer[index[i]]);
      }

      time_buffer2.clear();
      for (int i = 0; i < index_length; i++)
      {
        time_buffer2.push_back(yawrate_offset_status->time_buffer[index[i]] - yawrate_offset_status->time_buffer[index[0]]);
      }

      // Least-square
      double sum_xy, sum_x, sum_y, sum_x2;
      sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;
      for (int i = 0; i < index_length ; i++)
      {
        sum_xy += time_buffer2[i] * diff_buffer[i];
        sum_x += time_buffer2[i];
        sum_y += diff_buffer[i];
        sum_x2 += std::pow(time_buffer2[i], 2);
      }
      yawrate_offset->yawrate_offset = -1 * (index_length * sum_xy - sum_x * sum_y) / (index_length * sum_x2 - pow(sum_x, 2));
      yawrate_offset->status.enabled_status = true;
      yawrate_offset->status.estimate_status = true;
    }
  }

  if (!yawrate_offset->status.enabled_status)
  {
    yawrate_offset->yawrate_offset = yawrate_offset_stop.yawrate_offset;
  }

  if (std::fabs(yawrate_offset->yawrate_offset - yawrate_offset_stop.yawrate_offset) > yawrate_offset_parameter.outlier_threshold)
  {
    yawrate_offset->yawrate_offset = yawrate_offset_stop.yawrate_offset;
  }

}
