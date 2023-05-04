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
 * position.cpp
 * Author MapIV Sekino
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void position_estimate_(geometry_msgs::TwistStamped velocity,eagleye_msgs::StatusStamped velocity_status,eagleye_msgs::Distance distance,
  eagleye_msgs::Heading heading_interpolate_3rd,geometry_msgs::Vector3Stamped enu_vel,PositionParameter position_parameter,
  PositionStatus* position_status, eagleye_msgs::Position* enu_absolute_pos)
{
  int i;
  int estimated_number_max = position_parameter.estimated_interval/position_parameter.update_distance;
  int max_x_index, max_y_index;
  double enu_pos[3];
  double avg_x, avg_y, avg_z;
  double tmp_enu_pos_x, tmp_enu_pos_y, tmp_enu_pos_z;
  bool data_status, gnss_status;
  std::size_t index_length;
  std::size_t velocity_index_length;
  std::vector<double> base_enu_pos_x_buffer, base_enu_pos_y_buffer, base_enu_pos_z_buffer;
  std::vector<double> diff_x_buffer2, diff_y_buffer2, diff_z_buffer2;
  std::vector<double> base_enu_pos_x_buffer2,  base_enu_pos_y_buffer2, base_enu_pos_z_buffer2;
  std::vector<double> diff_x_buffer, diff_y_buffer, diff_z_buffer;
  std::vector<double>::iterator max_x, max_y;

  double enabled_data_ratio = position_parameter.gnss_rate / position_parameter.imu_rate * position_parameter.gnss_receiving_threshold;
  double remain_data_ratio = enabled_data_ratio * position_parameter.outlier_ratio_threshold;

  enu_pos[0] = position_status->enu_pos[0];
  enu_pos[1] = position_status->enu_pos[1];
  enu_pos[2] = position_status->enu_pos[2];

  if(!position_status->gnss_update_failure)
  {
    gnss_status = true;

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = enu_pos[0];
    pose.pose.position.y = enu_pos[1];
    pose.pose.position.z = enu_pos[2];

    heading_interpolate_3rd.heading_angle = fmod(heading_interpolate_3rd.heading_angle,2*M_PI);
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, (90* M_PI / 180)-heading_interpolate_3rd.heading_angle);
    transform.setRotation(q);

    tf::Transform transform2;
    tf::Quaternion q2(position_parameter.tf_gnss_rotation_x,position_parameter.tf_gnss_rotation_y,
      position_parameter.tf_gnss_rotation_z,position_parameter.tf_gnss_rotation_w);
    transform2.setOrigin(transform*tf::Vector3(-position_parameter.tf_gnss_translation_x, -position_parameter.tf_gnss_translation_y,
      -position_parameter.tf_gnss_translation_z));
    transform2.setRotation(transform*q2);

    tf::Vector3 tmp_pos;
    tmp_pos = transform2.getOrigin();

    enu_pos[0] = tmp_pos.getX();
    enu_pos[1] = tmp_pos.getY();
    enu_pos[2] = tmp_pos.getZ();
  }
  else
  {
    gnss_status = false;
  }

  if (heading_interpolate_3rd.status.estimate_status == true && velocity_status.status.enabled_status == true)
  {
    heading_interpolate_3rd.status.estimate_status = false; //in order to prevent being judged many times
    ++position_status->heading_estimate_status_count;
  }

  if(position_status->time_last != 0)
  {
    position_status->enu_relative_pos_x = position_status->enu_relative_pos_x + enu_vel.vector.x * (enu_vel.header.stamp.toSec() - position_status->time_last);
    position_status->enu_relative_pos_y = position_status->enu_relative_pos_y + enu_vel.vector.y * (enu_vel.header.stamp.toSec() - position_status->time_last);
    position_status->enu_relative_pos_z = position_status->enu_relative_pos_z + enu_vel.vector.z * (enu_vel.header.stamp.toSec() - position_status->time_last);
  }


  if (distance.distance-position_status->distance_last >= position_parameter.update_distance && gnss_status == true &&
    position_status->heading_estimate_status_count > 0)
  {

    if (position_status->estimated_number < estimated_number_max)
    {
      ++position_status->estimated_number;
    }
    else
    {
      position_status->estimated_number = estimated_number_max;
    }

    position_status->enu_pos_x_buffer.push_back(enu_pos[0]);
    position_status->enu_pos_y_buffer.push_back(enu_pos[1]);
    //enu_pos_z_buffer.push_back(enu_pos[2]);
    position_status->enu_pos_z_buffer.push_back(0);
    position_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);
    position_status->enu_relative_pos_x_buffer.push_back(position_status->enu_relative_pos_x);
    position_status->enu_relative_pos_y_buffer.push_back(position_status->enu_relative_pos_y);
    //position_status->enu_relative_pos_z_buffer.push_back(position_status->enu_relative_pos_z);
    position_status->enu_relative_pos_z_buffer.push_back(0);
    position_status->distance_buffer.push_back(distance.distance);

    data_status = true; //judgement that refreshed data

    if (position_status->distance_buffer.end() - position_status->distance_buffer.begin() > estimated_number_max)
    {
      position_status->enu_pos_x_buffer.erase(position_status->enu_pos_x_buffer.begin());
      position_status->enu_pos_y_buffer.erase(position_status->enu_pos_y_buffer.begin());
      position_status->enu_pos_z_buffer.erase(position_status->enu_pos_z_buffer.begin());
      position_status->correction_velocity_buffer.erase(position_status->correction_velocity_buffer.begin());
      position_status->enu_relative_pos_x_buffer.erase(position_status->enu_relative_pos_x_buffer.begin());
      position_status->enu_relative_pos_y_buffer.erase(position_status->enu_relative_pos_y_buffer.begin());
      position_status->enu_relative_pos_z_buffer.erase(position_status->enu_relative_pos_z_buffer.begin());
      position_status->distance_buffer.erase(position_status->distance_buffer.begin());
    }
    position_status->distance_last = distance.distance;
  }

  if (data_status == true)
  {

    if (distance.distance > position_parameter.estimated_interval && gnss_status == true &&
      velocity.twist.linear.x > position_parameter.moving_judgement_threshold && position_status->heading_estimate_status_count > 0)
    {
      std::vector<int> distance_index;
      std::vector<int> velocity_index;
      std::vector<int> index;

      for (i = 0; i < position_status->estimated_number; i++)
      {
        if (position_status->distance_buffer[position_status->estimated_number-1] - position_status->distance_buffer[i]  <= position_parameter.estimated_interval)
        {
          distance_index.push_back(i);

          if (position_status->correction_velocity_buffer[i] > position_parameter.moving_judgement_threshold)
          {
            velocity_index.push_back(i);
          }

        }
      }

      set_intersection(velocity_index.begin(), velocity_index.end(), distance_index.begin(), distance_index.end(),
                       inserter(index, index.end()));

      index_length = std::distance(index.begin(), index.end());
      velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

      if (index_length > velocity_index_length * enabled_data_ratio)
      {

        while (1)
        {
          index_length = std::distance(index.begin(), index.end());

          base_enu_pos_x_buffer.clear();
          base_enu_pos_y_buffer.clear();
          base_enu_pos_z_buffer.clear();

          for (i = 0; i < position_status->estimated_number; i++)
          {
            base_enu_pos_x_buffer.push_back(position_status->enu_pos_x_buffer[index[index_length-1]] -
              position_status->enu_relative_pos_x_buffer[index[index_length-1]] + position_status->enu_relative_pos_x_buffer[i]);
            base_enu_pos_y_buffer.push_back(position_status->enu_pos_y_buffer[index[index_length-1]] -
              position_status->enu_relative_pos_y_buffer[index[index_length-1]] + position_status->enu_relative_pos_y_buffer[i]);
            base_enu_pos_z_buffer.push_back(position_status->enu_pos_z_buffer[index[index_length-1]] - 
              position_status->enu_relative_pos_z_buffer[index[index_length-1]] + position_status->enu_relative_pos_z_buffer[i]);
          }

          diff_x_buffer2.clear();
          diff_y_buffer2.clear();
          diff_z_buffer2.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_x_buffer2.push_back(base_enu_pos_x_buffer[index[i]] - position_status->enu_pos_x_buffer[index[i]]);
            diff_y_buffer2.push_back(base_enu_pos_y_buffer[index[i]] - position_status->enu_pos_y_buffer[index[i]]);
            diff_z_buffer2.push_back(base_enu_pos_z_buffer[index[i]] - position_status->enu_pos_z_buffer[index[i]]);
          }

          avg_x = std::accumulate(diff_x_buffer2.begin(), diff_x_buffer2.end(), 0.0) / index_length;
          avg_y = std::accumulate(diff_y_buffer2.begin(), diff_y_buffer2.end(), 0.0) / index_length;
          avg_z = std::accumulate(diff_z_buffer2.begin(), diff_z_buffer2.end(), 0.0) / index_length;

          tmp_enu_pos_x = position_status->enu_pos_x_buffer[index[index_length - 1]] - avg_x;
          tmp_enu_pos_y = position_status->enu_pos_y_buffer[index[index_length - 1]] - avg_y;
          tmp_enu_pos_z = position_status->enu_pos_z_buffer[index[index_length - 1]] - avg_z;

          base_enu_pos_x_buffer2.clear();
          base_enu_pos_y_buffer2.clear();
          base_enu_pos_z_buffer2.clear();

          for (i = 0; i < position_status->estimated_number; i++)
          {
            base_enu_pos_x_buffer2.push_back(tmp_enu_pos_x - position_status->enu_relative_pos_x_buffer[index[index_length - 1]] +
              position_status->enu_relative_pos_x_buffer[i]);
            base_enu_pos_y_buffer2.push_back(tmp_enu_pos_y - position_status->enu_relative_pos_y_buffer[index[index_length - 1]] +
              position_status->enu_relative_pos_y_buffer[i]);
            base_enu_pos_z_buffer2.push_back(tmp_enu_pos_z - position_status->enu_relative_pos_z_buffer[index[index_length - 1]] +
              position_status->enu_relative_pos_z_buffer[i]);
          }

          diff_x_buffer.clear();
          diff_y_buffer.clear();
          diff_z_buffer.clear();

          for (i = 0; i < index_length; i++)
          {
            diff_x_buffer.push_back(fabsf(base_enu_pos_x_buffer2[index[i]] - position_status->enu_pos_x_buffer[index[i]]));
            diff_y_buffer.push_back(fabsf(base_enu_pos_y_buffer2[index[i]] - position_status->enu_pos_y_buffer[index[i]]));
            diff_z_buffer.push_back(fabsf(base_enu_pos_z_buffer2[index[i]] - position_status->enu_pos_z_buffer[index[i]]));
          }

          max_x = std::max_element(diff_x_buffer.begin(), diff_x_buffer.end());
          max_y = std::max_element(diff_y_buffer.begin(), diff_y_buffer.end());

          max_x_index = std::distance(diff_x_buffer.begin(), max_x);
          max_y_index = std::distance(diff_y_buffer.begin(), max_y);

          if(diff_x_buffer[max_x_index] < diff_y_buffer[max_y_index])
          {
            if (diff_x_buffer[max_x_index] > position_parameter.outlier_threshold)
            {
              index.erase(index.begin() + max_x_index);
            }
            else
            {
              break;
            }
          }
          else
          {
            if (diff_y_buffer[max_y_index] > position_parameter.outlier_threshold)
            {
              index.erase(index.begin() + max_y_index);
            }
            else
            {
              break;
            }
          }

          index_length = std::distance(index.begin(), index.end());
          velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

          if (index_length < velocity_index_length * remain_data_ratio)
          {
            break;
          }

        }

        index_length = std::distance(index.begin(), index.end());
        velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

        if (index_length >= velocity_index_length * remain_data_ratio)
        {

          std::vector<double> diff_x_buffer_for_covariance, diff_y_buffer_for_covariance, diff_z_buffer_for_covariance;
          for (i = 0; i < index_length; i++)
          {
            diff_x_buffer_for_covariance.push_back(base_enu_pos_x_buffer2[index[i]] - position_status->enu_pos_x_buffer[index[i]]);
            diff_y_buffer_for_covariance.push_back(base_enu_pos_y_buffer2[index[i]] - position_status->enu_pos_y_buffer[index[i]]);
            diff_z_buffer_for_covariance.push_back(base_enu_pos_z_buffer2[index[i]] - position_status->enu_pos_z_buffer[index[i]]);
          }

          avg_x = std::accumulate(diff_x_buffer_for_covariance.begin(), diff_x_buffer_for_covariance.end(), 0.0) / index_length;
          avg_y = std::accumulate(diff_y_buffer_for_covariance.begin(), diff_y_buffer_for_covariance.end(), 0.0) / index_length;
          avg_z = std::accumulate(diff_z_buffer_for_covariance.begin(), diff_z_buffer_for_covariance.end(), 0.0) / index_length;

          double cov_x, cov_y, cov_z;
          double square_sum_x = 0, square_sum_y = 0, square_sum_z = 0;
          for (i = 0; i < index_length; i++)
          {
            square_sum_x += (diff_x_buffer_for_covariance[i] - avg_x) * (diff_x_buffer_for_covariance[i] - avg_x);
            square_sum_y += (diff_y_buffer_for_covariance[i] - avg_y) * (diff_y_buffer_for_covariance[i] - avg_y);
            square_sum_z += (diff_z_buffer_for_covariance[i] - avg_z) * (diff_z_buffer_for_covariance[i] - avg_z);
          }
          cov_x = square_sum_x/index_length;
          cov_y = square_sum_y/index_length;
          cov_z = square_sum_z/index_length;

          if (index[index_length - 1] == position_status->estimated_number-1)
          {
            enu_absolute_pos->enu_pos.x = tmp_enu_pos_x;
            enu_absolute_pos->enu_pos.y = tmp_enu_pos_y;
            enu_absolute_pos->enu_pos.z = tmp_enu_pos_z;
            enu_absolute_pos->covariance[0] = cov_x + position_parameter.gnss_error_covariance; // [m^2]
            enu_absolute_pos->covariance[4] = cov_y + position_parameter.gnss_error_covariance; // [m^2]
            enu_absolute_pos->covariance[8] = cov_z + position_parameter.gnss_error_covariance; // [m^2]
            enu_absolute_pos->status.enabled_status = true;
            enu_absolute_pos->status.estimate_status = true;
          }
        }
      }
    }
  }
  position_status->time_last = enu_vel.header.stamp.toSec();
  data_status = false;
}

void position_estimate(rtklib_msgs::RtklibNav rtklib_nav,geometry_msgs::TwistStamped velocity,eagleye_msgs::StatusStamped velocity_status,
  eagleye_msgs::Distance distance,eagleye_msgs::Heading heading_interpolate_3rd,geometry_msgs::Vector3Stamped enu_vel,
  PositionParameter position_parameter, PositionStatus* position_status, eagleye_msgs::Position* enu_absolute_pos)
{
  double enu_pos[3];
  double ecef_pos[3];
  double ecef_base_pos[3];
  bool gnss_update_failure;

  if(enu_absolute_pos->ecef_base_pos.x == 0 && enu_absolute_pos->ecef_base_pos.y == 0 && enu_absolute_pos->ecef_base_pos.z == 0)
  {
    if (rtklib_nav.tow != 0)
    {
      enu_absolute_pos->ecef_base_pos.x = rtklib_nav.ecef_pos.x;
      enu_absolute_pos->ecef_base_pos.y = rtklib_nav.ecef_pos.y;
      enu_absolute_pos->ecef_base_pos.z = rtklib_nav.ecef_pos.z;

      if(position_parameter.ecef_base_pos_x != 0 && position_parameter.ecef_base_pos_y != 0 && position_parameter.ecef_base_pos_z != 0)
      {
        enu_absolute_pos->ecef_base_pos.x = position_parameter.ecef_base_pos_x;
        enu_absolute_pos->ecef_base_pos.y = position_parameter.ecef_base_pos_y;
        enu_absolute_pos->ecef_base_pos.z = position_parameter.ecef_base_pos_z;
      }
    }
    else
    {
      return;
    }

  }

  ecef_pos[0] = rtklib_nav.ecef_pos.x;
  ecef_pos[1] = rtklib_nav.ecef_pos.y;
  ecef_pos[2] = rtklib_nav.ecef_pos.z;
  ecef_base_pos[0] = enu_absolute_pos->ecef_base_pos.x;
  ecef_base_pos[1] = enu_absolute_pos->ecef_base_pos.y;
  ecef_base_pos[2] = enu_absolute_pos->ecef_base_pos.z;

  xyz2enu(ecef_pos, ecef_base_pos, enu_pos);

  if (!std::isfinite(enu_pos[0])||!std::isfinite(enu_pos[1])||!std::isfinite(enu_pos[2]))
  {
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    gnss_update_failure = true;
  }
  else
  {
    gnss_update_failure = false;
  }

  if (position_status->tow_last == rtklib_nav.tow || rtklib_nav.tow == 0)
  {
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    gnss_update_failure = true;
  }
  else
  {
    gnss_update_failure = false;
  }

  position_status->enu_pos[0] = enu_pos[0];
  position_status->enu_pos[1] = enu_pos[1];
  position_status->enu_pos[2] = enu_pos[2];
  position_status->gnss_update_failure = gnss_update_failure;
  position_status->tow_last = rtklib_nav.tow;

  position_estimate_(velocity, velocity_status, distance, heading_interpolate_3rd, enu_vel, position_parameter, position_status, enu_absolute_pos);
}

void position_estimate(nmea_msgs::Gpgga gga,geometry_msgs::TwistStamped velocity,eagleye_msgs::StatusStamped velocity_status,
  eagleye_msgs::Distance distance,eagleye_msgs::Heading heading_interpolate_3rd,geometry_msgs::Vector3Stamped enu_vel,
  PositionParameter position_parameter,PositionStatus* position_status,eagleye_msgs::Position* enu_absolute_pos)
{
  double llh_pos[3];
  double enu_pos[3];
  double ecef_pos[3];
  double ecef_base_pos[3];
  bool gnss_update_failure;

  llh_pos[0] = gga.lat *M_PI/180;
  llh_pos[1] = gga.lon *M_PI/180;
  llh_pos[2] = gga.alt + gga.undulation;
  
  llh2xyz(llh_pos,ecef_pos);

  if(enu_absolute_pos->ecef_base_pos.x == 0 && enu_absolute_pos->ecef_base_pos.y == 0 && enu_absolute_pos->ecef_base_pos.z == 0)
  {
    if (gga.header.stamp.toSec() != 0)
    {
      enu_absolute_pos->ecef_base_pos.x = ecef_pos[0];
      enu_absolute_pos->ecef_base_pos.y = ecef_pos[1];
      enu_absolute_pos->ecef_base_pos.z = ecef_pos[2];

      if(position_parameter.ecef_base_pos_x != 0 && position_parameter.ecef_base_pos_y != 0 && position_parameter.ecef_base_pos_z != 0)
      {
        enu_absolute_pos->ecef_base_pos.x = position_parameter.ecef_base_pos_x;
        enu_absolute_pos->ecef_base_pos.y = position_parameter.ecef_base_pos_y;
        enu_absolute_pos->ecef_base_pos.z = position_parameter.ecef_base_pos_z;
      }
    }
    else
    {
      return;
    }

  }

  ecef_base_pos[0] = enu_absolute_pos->ecef_base_pos.x;
  ecef_base_pos[1] = enu_absolute_pos->ecef_base_pos.y;
  ecef_base_pos[2] = enu_absolute_pos->ecef_base_pos.z;

  xyz2enu(ecef_pos, ecef_base_pos, enu_pos);

  if (!std::isfinite(enu_pos[0])||!std::isfinite(enu_pos[1])||!std::isfinite(enu_pos[2]))
  {
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    gnss_update_failure = true;
  }
  else
  {
    gnss_update_failure = false;
  }

  if (position_status->nmea_time_last == gga.header.stamp.toSec() || enu_vel.header.stamp.toSec() == 0)
  {
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    gnss_update_failure = true;
  }
  else
  {
    gnss_update_failure = false;
  }

  position_status->enu_pos[0] = enu_pos[0];
  position_status->enu_pos[1] = enu_pos[1];
  position_status->enu_pos[2] = enu_pos[2];
  position_status->gnss_update_failure = gnss_update_failure;
  position_status->nmea_time_last = gga.header.stamp.toSec();

  position_estimate_(velocity, velocity_status, distance, heading_interpolate_3rd, enu_vel, position_parameter, position_status, enu_absolute_pos);
}
