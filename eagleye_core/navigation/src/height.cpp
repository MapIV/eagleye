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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#define g 9.80665

void pitching_estimate(const sensor_msgs::msg::Imu imu, const nmea_msgs::msg::Gpgga gga, const geometry_msgs::msg::TwistStamped velocity,
  const eagleye_msgs::msg::Distance distance,const HeightParameter height_parameter,HeightStatus* height_status,eagleye_msgs::msg::Height* height,
  eagleye_msgs::msg::Pitching* pitching,eagleye_msgs::msg::AccXOffset* acc_x_offset,eagleye_msgs::msg::AccXScaleFactor* acc_x_scale_factor)
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
  bool data_status = false;
  int i;
  int data_num = 0;
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
  std::size_t index_length;
  std::size_t velocity_index_length;
  std::size_t distance_index_length;
  std::vector<double>::iterator max_height;

  int buffer_erase_count = 0;

  double moving_average_buffer_number = height_parameter.moving_average_time * height_parameter.imu_rate;
  double enabled_data_ratio = height_parameter.gnss_rate / height_parameter.imu_rate * height_parameter.gnss_receiving_threshold;
  double remain_data_ratio = enabled_data_ratio * height_parameter.outlier_ratio_threshold;

  rclcpp::Time ros_clock(gga.header.stamp);
  rclcpp::Time ros_clock2(imu.header.stamp);
  auto gga_time = ros_clock.seconds();
  auto imu_time = ros_clock2.seconds();

/// GNSS FLAG ///
  if (height_status->gga_time_last == gga_time)
  {
    gnss_status = false;
    gnss_height = 0.0;
    gps_quality = 0;
    height_status->gga_time_last = gga_time;
  }
  else
  {
    gnss_status = true;
    gnss_height = gga.alt + gga.undulation;
    gps_quality = gga.gps_qual;
    height_status->gga_time_last = gga_time;
  }

  height_status->flag_reliability = false;

///  relative_height  ///
  if (velocity.twist.linear.x > 0 && height_status->time_last != 0)
  {
    height_status->relative_height_G += imu.linear_acceleration.x * velocity.twist.linear.x*(imu_time-height_status->time_last)/g;
    height_status->relative_height_diffvel += - (velocity.twist.linear.x-height_status->correction_velocity_x_last) * velocity.twist.linear.x/g;
    height_status->relative_height_offset += velocity.twist.linear.x*(imu_time-height_status->time_last)/g;
    correction_relative_height = height_status->relative_height_G + height_status->relative_height_offset + height_status->relative_height_diffvel;
  }

///  buffering  ///
  if (distance.distance-height_status->distance_last >= height_parameter.update_distance && gnss_status == true && gps_quality == 4)
  {
    height_status->height_buffer.push_back(gnss_height);
    height_status->relative_height_G_buffer.push_back(height_status->relative_height_G);
    height_status->relative_height_diffvel_buffer.push_back(height_status->relative_height_diffvel);
    height_status->relative_height_offset_buffer.push_back(height_status->relative_height_offset);
    height_status->correction_relative_height_buffer.push_back(correction_relative_height);
    height_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);
    height_status->distance_buffer.push_back(distance.distance);
    data_status = true;

    if (height_status->distance_buffer[height_status->data_number-1] - height_status->distance_buffer[0] > height_parameter.estimated_maximum_interval)
    {
      height_status->height_buffer.erase(height_status->height_buffer.begin());
      height_status->relative_height_G_buffer.erase(height_status->relative_height_G_buffer.begin());
      height_status->relative_height_diffvel_buffer.erase(height_status->relative_height_diffvel_buffer.begin());
      height_status->relative_height_offset_buffer.erase(height_status->relative_height_offset_buffer.begin());
      height_status->correction_relative_height_buffer.erase(height_status->correction_relative_height_buffer.begin());
      height_status->correction_velocity_buffer.erase(height_status->correction_velocity_buffer.begin());
      height_status->distance_buffer.erase(height_status->distance_buffer.begin());
      height_status->acceleration_SF_estimate_status = true;
    }

    height_status->data_number = height_status->distance_buffer.size();

    if (height_status->distance_buffer[height_status->data_number-1]- height_status->distance_buffer[0] > height_parameter.estimated_minimum_interval)
    {
      height_status->estimate_start_status = true;
    }
    else
    {
      height_status->estimate_start_status = false;
    }

    height_status->distance_last = distance.distance;

  }

  height_status->data_number = height_status->distance_buffer.size();

///  acc_x error estimate   ///
  if (height_status->estimate_start_status == true && gnss_status == true && data_status == true)
  {

///  Explanation  ///

    A = 0.0;
    B = 0.0;
    C = 0.0;
    D = 0.0;
    E = 0.0;

    if (height_status->acceleration_SF_estimate_status == true)
    {
      for (i = 0; i < height_status->data_number; i++)
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
      height_status->acceleration_offset_linear_x_last = (2*A*D - C*B)/(C*C - 4*A*E);
      height_status->acceleration_SF_linear_x_last = (2*E*B - C*D)/(C*C - 4*A*E);

      acc_x_offset->status.enabled_status = true;
      acc_x_offset->status.estimate_status = true;
      acc_x_scale_factor->status.enabled_status = true;
      acc_x_scale_factor->status.estimate_status = true;
    }
    else
    {
      for (i = 0; i < height_status->data_number; i++)
      {
        diff_height = height_status->height_buffer[i] - height_status->height_buffer[0];
        diff_relative_height = (height_status->relative_height_G_buffer[i] + height_status->relative_height_diffvel_buffer[i])- (height_status->relative_height_G_buffer[0] + height_status->relative_height_diffvel_buffer[0]);
        diff_relative_height_offset = height_status->relative_height_offset_buffer[i] - height_status->relative_height_offset_buffer[0];
        A += diff_relative_height_offset * diff_relative_height_offset;
        B += 2 * diff_relative_height_offset * (diff_height - diff_relative_height);
      }
      height_status->acceleration_offset_linear_x_last = B/A/2;
      height_status->acceleration_SF_linear_x_last = 1;
      acc_x_offset->status.enabled_status = true;
      acc_x_offset->status.estimate_status = true;
      acc_x_scale_factor->status.enabled_status = false;
      acc_x_scale_factor->status.estimate_status = false;
    }

    for (i = 0; i < height_status->data_number; i++)
    {
      height_status->correction_relative_height_buffer[i] = height_status->acceleration_SF_linear_x_last * height_status->relative_height_G_buffer[i] + height_status->relative_height_diffvel_buffer[i] + height_status->acceleration_offset_linear_x_last * height_status->relative_height_offset_buffer[i];
    }
  }

///  height estimate  ///
  if (height_status->estimate_start_status == true)
  {
    if (distance.distance > height_parameter.estimated_minimum_interval && gnss_status == true && gps_quality ==4 && data_status == true &&
      velocity.twist.linear.x > height_parameter.moving_judgment_threshold )
    {
      height_status->correction_relative_height_buffer2.clear();
      height_status->height_buffer2.clear();
      for (i = 0; i < height_status->data_number; i++)
      {
        height_status->correction_relative_height_buffer2.push_back(height_status->correction_relative_height_buffer[i]);
        height_status->height_buffer2.push_back(height_status->height_buffer[i]);
      }

      std::vector<int> distance_index;
      std::vector<int> velocity_index;
      std::vector<int> index;

      for (i = 0; i < height_status->data_number; i++)
      {
        if (height_status->distance_buffer[height_status->data_number-1] - height_status->distance_buffer[i]  <= height_parameter.estimated_minimum_interval)
        {
          distance_index.push_back(i);

          if (height_status->correction_velocity_buffer[i] > height_parameter.moving_judgment_threshold)
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
      std::vector<int> erase_number;

      if (index_length > velocity_index_length * enabled_data_ratio)
      {

        while (1)
        {
          index_length = std::distance(index.begin(), index.end());
          base_height_buffer.clear();

          for (i = 0; i < height_status->data_number; i++)
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

          for (i = 0; i < height_status->data_number; i++)
          {
            base_height_buffer2.push_back(tmp_height - height_status->correction_relative_height_buffer2[index[index_length - 1]] + height_status->correction_relative_height_buffer2[i]);
          }

          diff_height_buffer.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_height_buffer.push_back(std::fabs(base_height_buffer2[index[i]] - height_status->height_buffer2[index[i]]));
          }

          max_height = std::max_element(diff_height_buffer.begin(), diff_height_buffer.end());
          max_height_index = std::distance(diff_height_buffer.begin(), max_height);

          if (diff_height_buffer[max_height_index] > height_parameter.outlier_threshold)
          {
            if (height_status->height_estimate_start_status != true)
            {
              erase_number.push_back(index[max_height_index]);
              buffer_erase_count =  buffer_erase_count + 1;
            }
            else if (index[max_height_index] == height_status->data_number-1)
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

          if (index_length < velocity_index_length * remain_data_ratio)
          {
            break;
          }
        }

        if(height_status->height_estimate_start_status != true)
        {
          std::sort(erase_number.begin(), erase_number.end(), std::greater<int>() );
          for(i=0;i<buffer_erase_count;i++)
          {
            height_status->height_buffer.erase(height_status->height_buffer.begin() + erase_number[i]);
            height_status->relative_height_G_buffer.erase(height_status->relative_height_G_buffer.begin() + erase_number[i]);
            height_status->relative_height_diffvel_buffer.erase(height_status->relative_height_diffvel_buffer.begin() + erase_number[i]);
            height_status->relative_height_offset_buffer.erase(height_status->relative_height_offset_buffer.begin() + erase_number[i]);
            height_status->correction_relative_height_buffer.erase(height_status->correction_relative_height_buffer.begin() + erase_number[i]);
            height_status->correction_velocity_buffer.erase(height_status->correction_velocity_buffer.begin() + erase_number[i]);
            height_status->distance_buffer.erase(height_status->distance_buffer.begin() + erase_number[i]);
          }
        }



        height_status->height_estimate_start_status = true;

        index_length = std::distance(index.begin(), index.end());
        velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

        if (index_length >= velocity_index_length * remain_data_ratio)
        {
          if (index[index_length - 1] == height_status->data_number-1)
          {
            height_status->height_last = tmp_height;
            height->status.enabled_status = true;
            height->status.estimate_status = true;
            height_status->flag_reliability = true;
          }
          else
          {
            height_status->height_last = tmp_height + (height_status->correction_relative_height_buffer2[height_status->data_number - 1]
              - height_status->correction_relative_height_buffer2[index[index_length - 1]]);
            height->status.enabled_status = true;
            height->status.estimate_status = true;
            height_status->flag_reliability = false;
          }
        }
      }
    }
    else
    {
      height_status->height_last += ((imu.linear_acceleration.x * height_status->acceleration_SF_linear_x_last + height_status->acceleration_offset_linear_x_last)
      - (velocity.twist.linear.x-height_status->correction_velocity_x_last)/(imu_time-height_status->time_last))
      * velocity.twist.linear.x*(imu_time-height_status->time_last)/g;
      height->status.enabled_status = true;
      height->status.estimate_status = false;
    }
  }

///  pitch  ///
  correction_acceleration_linear_x = imu.linear_acceleration.x * height_status->acceleration_SF_linear_x_last + height_status->acceleration_offset_linear_x_last;
  height_status->acc_buffer.push_back((correction_acceleration_linear_x - (velocity.twist.linear.x-height_status->correction_velocity_x_last)/(imu_time-height_status->time_last)));
  data_num_acc = height_status->acc_buffer.size();

  if (data_num_acc > moving_average_buffer_number)
  {
    height_status->acc_buffer.erase(height_status->acc_buffer.begin());
    data_num_acc--;
  }

  if (data_num_acc >= moving_average_buffer_number && height_status->estimate_start_status == true)
  {
    sum_acc = 0;
    for (i = 0; i < data_num_acc; i++)
    {
      sum_acc += height_status->acc_buffer[i];
    }
    mean_acc = sum_acc / data_num_acc;

    if (std::abs(mean_acc/g) < 1)
    {
      tmp_pitch = std::asin(mean_acc/g);
      pitching->status.enabled_status = true;
      pitching->status.estimate_status = true;
    }
    else
    {
      tmp_pitch = height_status->pitching_angle_last;
      pitching->status.enabled_status = false;
      pitching->status.estimate_status = true;
    }

  }
  else
  {
    tmp_pitch = 0;
    pitching->status.enabled_status = false;
    pitching->status.estimate_status = false;
  }

  acc_x_offset->acc_x_offset = height_status->acceleration_offset_linear_x_last;
  acc_x_scale_factor->acc_x_scale_factor = height_status->acceleration_SF_linear_x_last;
  height->height = height_status->height_last;
  pitching->pitching_angle = tmp_pitch;

  height_status->time_last = imu_time;
  height_status->correction_velocity_x_last = velocity.twist.linear.x;
  height_status->pitching_angle_last = tmp_pitch;
}
