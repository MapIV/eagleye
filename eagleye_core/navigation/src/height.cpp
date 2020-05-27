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
 * height.cpp
 * Author MapIV  Takanose
 */

#include "navigation.hpp"

#define g 9.80665

void pitching_estimate(const sensor_msgs::Imu imu,const sensor_msgs::NavSatFix fix,const eagleye_msgs::VelocityScaleFactor velocity_scale_factor,const eagleye_msgs::Distance distance,const HeightParameter height_parameter,HeightStatus* height_status,eagleye_msgs::Height* height,eagleye_msgs::Pitching* pitching,eagleye_msgs::AccXOffset* acc_x_offset,eagleye_msgs::AccXScaleFactor* acc_x_scale_factor)
{
  int gps_quality = 0;
  double gnss_height = 0.0;
  double diff_height = 0.0;
  double diff_relative_height = 0.0;
  double diff_relative_height_G = 0.0;
  double diff_relative_height_diffvel = 0.0;
  double diff_relative_height_offset = 0.0;
  double correction_relative_height = 0.0;
  bool gnss_status;
  int i;
  int data_num;
  int max_height_index;
  double A, B, C, D, E;
  double avg_height;
  double tmp_height;
  double tmp_pitch;
  int data_num_acc = 0;
  double sum_acc = 0;
  double mean_acc = 0;
  double pitch = 0;
  double correction_acceleration_linear_x = 0;
  double acceleration_offset_linear_x;
  double acceleration_SF_linear_x;
  bool estimate_start_status = false;
  bool acceleration_SF_estimate_status = false;

  std::size_t index_length;
  std::size_t velocity_index_length;
  std::size_t distance_index_length;

  std::vector<double>::iterator max_height;

/// GNSS FLAG ///
  if (height_status->fix_time_last == fix.header.stamp.toSec())
  {
    gnss_status = false;
    gnss_height = 0.0;
    gps_quality = 0;
    height_status->fix_time_last = fix.header.stamp.toSec();
  }
  else
  {
    gnss_status = true;
    gnss_height = fix.altitude;
    gps_quality = fix.status.status;
    height_status->fix_time_last = fix.header.stamp.toSec();
  }

///  relative_height  ///
  if (velocity_scale_factor.correction_velocity.linear.x > 0 && height_status->time_last != 0)
  {
    height_status->relative_height_G += imu.linear_acceleration.x * velocity_scale_factor.correction_velocity.linear.x*(imu.header.stamp.toSec()-height_status->time_last)/g;
    height_status->relative_height_diffvel += - (velocity_scale_factor.correction_velocity.linear.x-height_status->correction_velocity_x_last) * velocity_scale_factor.correction_velocity.linear.x/g;
    height_status->relative_height_offset += velocity_scale_factor.correction_velocity.linear.x*(imu.header.stamp.toSec()-height_status->time_last)/g;
    correction_relative_height = height_status->relative_height_G + height_status->relative_height_offset + height_status->relative_height_diffvel;
  }

///  buffering  ///
  if (distance.distance-height_status->distance_last >= height_parameter.separation_distance && gnss_status == true && gps_quality == 1)
  {
    height_status->height_buffer.push_back(gnss_height);
    height_status->relative_height_G_buffer.push_back(height_status->relative_height_G);
    height_status->relative_height_diffvel_buffer.push_back(height_status->relative_height_diffvel);
    height_status->relative_height_offset_buffer.push_back(height_status->relative_height_offset);
    height_status->correction_relative_height_buffer.push_back(correction_relative_height);
    height_status->correction_velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);
    height_status->distance_buffer.push_back(distance.distance);
    data_num = height_status->distance_buffer.size();

    if (height_status->distance_buffer[data_num-1]- height_status->distance_buffer[0] > height_parameter.estimated_distance)
    {
      estimate_start_status = true;
    }

    if (height_status->distance_buffer[data_num-1] - height_status->distance_buffer[0] > height_parameter.estimated_distance_max)
    {
      height_status->height_buffer.erase(height_status->height_buffer.begin());
      height_status->relative_height_G_buffer.erase(height_status->relative_height_G_buffer.begin());
      height_status->relative_height_diffvel_buffer.erase(height_status->relative_height_diffvel_buffer.begin());
      height_status->relative_height_offset_buffer.erase(height_status->relative_height_offset_buffer.begin());
      height_status->correction_relative_height_buffer.erase(height_status->correction_relative_height_buffer.begin());
      height_status->correction_velocity_buffer.erase(height_status->correction_velocity_buffer.begin());
      height_status->distance_buffer.erase(height_status->distance_buffer.begin());
      data_num = height_status->distance_buffer.size();
      acceleration_SF_estimate_status = true;
    }

    height_status->distance_last = distance.distance;
  }


///  acc_x error estimate   ///
  if (estimate_start_status == true && gnss_status == true)
  {

///  Explanation  ///

    A = 0.0;
    B = 0.0;
    C = 0.0;
    D = 0.0;
    E = 0.0;

    if (acceleration_SF_estimate_status == true)
    {
      for (i = 0; i < data_num; i++)
      {
        diff_height = height_status->height_buffer[i] - height_status->height_buffer[0];
        diff_relative_height_G = height_status->relative_height_G_buffer[i] - height_status->relative_height_G_buffer[0];
        diff_relative_height_diffvel = height_status->relative_height_diffvel_buffer[i] - height_status->relative_height_diffvel_buffer[0];
        diff_relative_height_offset = height_status->relative_height_offset_buffer[i] - height_status->relative_height_offset_buffer[0];
        A += diff_relative_height_G * diff_relative_height_G;
        B += 2 * diff_relative_height_G * (diff_relative_height_diffvel - diff_height);
        C += 2 * diff_relative_height_G * diff_relative_height_offset;
        D += 2 * diff_relative_height_offset * (diff_relative_height_diffvel - diff_height);
        E += diff_relative_height_offset * diff_relative_height_offset;
      }
      acceleration_offset_linear_x = (2*A*D - C*B)/(C*C - 4*A*E);
      acceleration_SF_linear_x = (2*E*B - C*D)/(C*C - 4*A*E);

      acc_x_offset->status.enabled_status = true;
      acc_x_offset->status.estimate_status = true;
      acc_x_scale_factor->status.enabled_status = true;
      acc_x_scale_factor->status.estimate_status = true;
    }
    else if (acceleration_SF_estimate_status == false)
    {
      for (i = 0; i < data_num; i++)
      {
        diff_height = height_status->height_buffer[i] - height_status->height_buffer[0];
        diff_relative_height = (height_status->relative_height_G_buffer[i] + height_status->relative_height_diffvel_buffer[i])- (height_status->relative_height_G_buffer[0] + height_status->relative_height_diffvel_buffer[0]);
        diff_relative_height_offset = height_status->relative_height_offset_buffer[i] - height_status->relative_height_offset_buffer[0];
        A += diff_relative_height_offset * diff_relative_height_offset;
        B += 2 * diff_relative_height_offset * (diff_height - diff_relative_height);
      }
      acceleration_offset_linear_x = B/A/2;

      acc_x_offset->status.enabled_status = true;
      acc_x_offset->status.estimate_status = true;
      acc_x_scale_factor->status.enabled_status = false;
      acc_x_scale_factor->status.estimate_status = false;
    }
    else
    {
      acc_x_offset->status.enabled_status = false;
      acc_x_offset->status.estimate_status = false;
      acc_x_scale_factor->status.enabled_status = false;
      acc_x_scale_factor->status.estimate_status = false;
    }

    for (i = 0; i < data_num; i++)
    {
      height_status->correction_relative_height_buffer[i] = acceleration_SF_linear_x * height_status->relative_height_G_buffer[i] + height_status->relative_height_diffvel_buffer[i] + acceleration_offset_linear_x * height_status->relative_height_offset_buffer[i];
    }
  }

///  height estimate  ///
  if (estimate_start_status == true)
  {
    if (distance.distance > height_parameter.estimated_distance && gnss_status == true && gps_quality == 1 && velocity_scale_factor.correction_velocity.linear.x > height_parameter.estimated_velocity_threshold )
    {
      height_status->correction_relative_height_buffer2.clear();
      height_status->height_buffer2.clear();
      for (i = 0; i < data_num; i++)
      {
        height_status->correction_relative_height_buffer2.push_back(height_status->correction_relative_height_buffer[i]);
        height_status->height_buffer2.push_back(height_status->height_buffer[i]);
      }

      std::vector<int> distance_index;
      std::vector<int> velocity_index;
      std::vector<int> index;

      for (i = 0; i < data_num; i++)
      {
        if (height_status->distance_buffer[data_num-1] - height_status->distance_buffer[i]  <= height_parameter.estimated_distance)
        {
          distance_index.push_back(i);

          if (height_status->correction_velocity_buffer[i] > height_parameter.estimated_velocity_threshold)
          {
            velocity_index.push_back(i);
          }
        }
      }

      set_intersection(velocity_index.begin(), velocity_index.end(), distance_index.begin(), distance_index.end(),
                       inserter(index, index.end()));

      index_length = std::distance(index.begin(), index.end());
      velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

      std::vector<double> base_height_buffer;
      std::vector<double> base_height_buffer2;
      std::vector<double> diff_height_buffer;
      std::vector<double> diff_height_buffer2;

      if (index_length > velocity_index_length * height_parameter.estimated_velocity_coefficient)
      {
        while (1)
        {
          index_length = std::distance(index.begin(), index.end());

          base_height_buffer.clear();

          for (i = 0; i < data_num; i++)
          {
            base_height_buffer.push_back(height_status->height_buffer2[index[index_length-1]] - height_status->correction_relative_height_buffer2[index[index_length-1]] +
                                height_status->correction_relative_height_buffer2[i]);
          }

          diff_height_buffer2.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_height_buffer2.push_back(base_height_buffer[index[i]] - height_status->height_buffer2[index[i]]);
          }

          avg_height = std::accumulate(diff_height_buffer2.begin(), diff_height_buffer2.end(), 0.0) / index_length;

          tmp_height = height_status->height_buffer2[index[index_length - 1]] - avg_height;

          base_height_buffer2.clear();

          for (i = 0; i < data_num; i++)
          {
            base_height_buffer2.push_back(tmp_height - height_status->correction_relative_height_buffer2[index[index_length - 1]] + height_status->correction_relative_height_buffer2[i]);
          }

          diff_height_buffer.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_height_buffer.push_back(fabsf(base_height_buffer2[index[i]] - height_status->height_buffer2[index[i]]));
          }

          max_height = std::max_element(diff_height_buffer.begin(), diff_height_buffer.end());

          max_height_index = std::distance(diff_height_buffer.begin(), max_height);

          if (diff_height_buffer[max_height_index] > height_parameter.outlier_threshold)
          {
            if (index[max_height_index] == data_num-1 || height_status->height_estimate_start_status != true)
            {
              height_status->height_buffer.erase(height_status->height_buffer.begin() + index[max_height_index]);
              height_status->relative_height_G_buffer.erase(height_status->relative_height_G_buffer.begin() + index[max_height_index]);
              height_status->relative_height_diffvel_buffer.erase(height_status->relative_height_diffvel_buffer.begin() + index[max_height_index]);
              height_status->relative_height_offset_buffer.erase(height_status->relative_height_offset_buffer.begin() + index[max_height_index]);
              height_status->correction_relative_height_buffer.erase(height_status->correction_relative_height_buffer.begin() + index[max_height_index]);
              height_status->correction_velocity_buffer.erase(height_status->correction_velocity_buffer.begin() + index[max_height_index]);
              height_status->distance_buffer.erase(height_status->distance_buffer.begin() + index[max_height_index]);
            }
            index.erase(index.begin() + max_height_index);
          }
          else
          {
            break;
          }

          index_length = std::distance(index.begin(), index.end());
          velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

          if (index_length < velocity_index_length * height_parameter.estimated_height_coefficient)
          {
            break;
          }
        }

        height_status->height_estimate_start_status = true;

        index_length = std::distance(index.begin(), index.end());
        velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

        if (index_length >= velocity_index_length * height_parameter.estimated_height_coefficient)
        {
          if (index[index_length - 1] == data_num-1)
          {
            height_status->height_last = tmp_height;
            height->status.enabled_status = true;
            height->status.estimate_status = true;
          }
          else
          {
            height_status->height_last = tmp_height + (height_status->correction_relative_height_buffer2[data_num - 1]
              - height_status->correction_relative_height_buffer2[index[index_length - 1]]);
            height->status.enabled_status = true;
            height->status.estimate_status = true;
          }
        }
      }
    }
    else
    {
      height_status->height_last += ((imu.linear_acceleration.x * acceleration_SF_linear_x + acceleration_offset_linear_x)
      - (velocity_scale_factor.correction_velocity.linear.x-height_status->correction_velocity_x_last)/(imu.header.stamp.toSec()-height_status->time_last))
      * velocity_scale_factor.correction_velocity.linear.x*(imu.header.stamp.toSec()-height_status->time_last)/g;
      height->status.enabled_status = true;
      height->status.estimate_status = false;
    }
  }

///  pitch  ///
  correction_acceleration_linear_x = imu.linear_acceleration.x * acceleration_SF_linear_x + acceleration_offset_linear_x;
  height_status->acc_buffer.push_back((correction_acceleration_linear_x - (velocity_scale_factor.correction_velocity.linear.x-height_status->correction_velocity_x_last)/(imu.header.stamp.toSec()-height_status->time_last)));
  data_num_acc = height_status->acc_buffer.size();

  if (data_num_acc > height_parameter.average_num)
  {
    height_status->acc_buffer.erase(height_status->acc_buffer.begin());
    data_num_acc--;

    sum_acc = 0;
    for (i = 0; i < data_num_acc; i++)
    {
      sum_acc += height_status->acc_buffer[i];
    }
    mean_acc = sum_acc / data_num_acc;
    tmp_pitch = asin(mean_acc/g);
    pitching->status.enabled_status = true;
    pitching->status.estimate_status = true;
  }
  else
  {
    tmp_pitch = 0;
    pitching->status.enabled_status = false;
    pitching->status.estimate_status = false;
  }

  acc_x_offset->acc_x_offset = acceleration_offset_linear_x;
  acc_x_scale_factor->acc_x_scale_factor = acceleration_SF_linear_x;
  height->height = height_status->height_last;
  pitching->pitching_angle = tmp_pitch;

  height_status->time_last = imu.header.stamp.toSec();
  height_status->correction_velocity_x_last = velocity_scale_factor.correction_velocity.linear.x;
}
