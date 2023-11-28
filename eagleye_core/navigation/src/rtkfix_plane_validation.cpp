// Copyright (c) 2023, Map IV, Inc.
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
 * rtkfix_plane_validation.cpp
 * Author MapIV Takanose
 */

#include "eagleye_navigation/rtkfix_plane_validation.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"

#include <numeric>

RtkfixPlaneValidationEstimator::RtkfixPlaneValidationEstimator()
{
  // Initialization
  is_first_epoch_ = true;
  gga_time_last_ = 0;
}

void RtkfixPlaneValidationEstimator::setParameter(const RtkfixPlaneValidationParameter& param)
{
  // Param
  param_ = param;
  // std::cout << "***debug-code: param_.validation_minimum_interval: " << param_.validation_minimum_interval << std::endl;
  // std::cout << "***debug-code: param_.validation_maximum_interval: " << param_.validation_maximum_interval << std::endl;
  // std::cout << "***debug-code: param_.outlier_threshold: " << param_.outlier_threshold << std::endl;
}

RtkfixPlaneValidationStatus RtkfixPlaneValidationEstimator::estimate(const nmea_msgs::msg::Gpgga gga, const eagleye_msgs::msg::Distance distance, const geometry_msgs::msg::Vector3Stamped enu_vel)
{
  RtkfixPlaneValidationStatus status;
  status.is_estimation_started = false;
  status.is_estimated_now = false;
  status.is_estimation_reliable = false;

  // buffering
  rclcpp::Time gga_timestamp_now(gga.header.stamp);
  double gga_time_now = gga_timestamp_now.seconds();
  bool gga_update_flag = false;
  if(gga_time_last_ != gga_time_now && gga.gps_qual == 4)
  {
    gga_update_flag = true;
    gga_time_last_ = gga_time_now;
  }

  bool reliable_status = false;
  bool unreliable_status = false;
  gga_buffer_.push_back(gga);
  gga_update_buffer_.push_back(gga_update_flag);
  distance_buffer_.push_back(distance.distance);
  enu_vel_buffer_.push_back(enu_vel);
  reliable_status_buffer_.push_back(reliable_status);
  unreliable_status_buffer_.push_back(unreliable_status);

  if(is_first_epoch_)
  {
    is_first_epoch_ = false;
    return status;
  }

  double distance_start = distance_buffer_.front();
  double distance_end = distance_buffer_.back();

  if(distance_end - distance_start < param_.validation_minimum_interval) return status;

  while(1)
  {
    distance_start = distance_buffer_.front();
    distance_end = distance_buffer_.back();
    if(distance_end - distance_start < param_.validation_maximum_interval) break;

    gga_buffer_.erase(gga_buffer_.begin());
    gga_update_buffer_.erase(gga_update_buffer_.begin());
    distance_buffer_.erase(distance_buffer_.begin());
    enu_vel_buffer_.erase(enu_vel_buffer_.begin());
    reliable_status_buffer_.erase(reliable_status_buffer_.begin());
    unreliable_status_buffer_.erase(unreliable_status_buffer_.begin());
  }

  // Search for past Fix solutions
  if(!gga_update_flag) return status;

  int buffer_length = distance_buffer_.size();
  int index_start;
  for(int iter = 0; iter < buffer_length; iter++)
  {
    index_start = buffer_length-1 -iter;
    if(distance_end - distance_buffer_[index_start] < param_.validation_minimum_interval) continue;
    if(gga_update_buffer_[index_start] && !unreliable_status_buffer_[index_start]) break;
  }
  if(!gga_update_buffer_[index_start]) return status;

  // Fix solution verification by dead reckoning
  double init_llh_pos[3];
  double init_enu_pos[3];
  double init_ecef_base_pos[3];
  init_llh_pos[0] = gga_buffer_[index_start].lat *M_PI/180;
  init_llh_pos[1] = gga_buffer_[index_start].lon *M_PI/180;
  init_llh_pos[2] = gga_buffer_[index_start].alt + gga_buffer_[index_start].undulation;
  llh2xyz(init_llh_pos,init_ecef_base_pos);
  xyz2enu(init_ecef_base_pos, init_ecef_base_pos, init_enu_pos);

  double relative_position_x = init_enu_pos[0];
  double relative_position_y = init_enu_pos[1];
  for(int iter = index_start+1; iter < buffer_length; iter++)
  {
    rclcpp::Time t1(enu_vel_buffer_[iter].header.stamp);
    rclcpp::Time t0(enu_vel_buffer_[iter-1].header.stamp);
    double dt = t1.seconds() - t0.seconds();
    relative_position_x = relative_position_x + enu_vel_buffer_[iter].vector.x * dt;
    relative_position_y = relative_position_y + enu_vel_buffer_[iter].vector.y * dt;
  }

  double target_llh_pos[3];
  double target_enu_pos[3];
  double target_ecef_pos[3];
  target_llh_pos[0] = gga_buffer_[buffer_length-1].lat *M_PI/180;
  target_llh_pos[1] = gga_buffer_[buffer_length-1].lon *M_PI/180;
  target_llh_pos[2] = gga_buffer_[buffer_length-1].alt + gga_buffer_[buffer_length-1].undulation;
  llh2xyz(target_llh_pos,target_ecef_pos);
  xyz2enu(target_ecef_pos, init_ecef_base_pos, target_enu_pos);

  double residual_x = target_enu_pos[0] - relative_position_x;
  double residual_y = target_enu_pos[1] - relative_position_y;
  double residual_2d = std::sqrt(residual_x*residual_x + residual_y*residual_y);

  if(residual_2d > param_.outlier_threshold)
  {
    unreliable_status_buffer_[buffer_length-1] = true;
    return status;
  } 

  if(reliable_status_buffer_[index_start])
  {
    status.fix_msg.header = gga_buffer_[buffer_length-1].header;
    status.fix_msg.latitude = gga_buffer_[buffer_length-1].lat;
    status.fix_msg.longitude = gga_buffer_[buffer_length-1].lon;
    status.fix_msg.altitude = gga_buffer_[buffer_length-1].alt + gga_buffer_[buffer_length-1].undulation;
    status.is_estimation_reliable = true;
  }
  reliable_status_buffer_[buffer_length-1] = true;

  // Debug 
  // rclcpp::Time time_now(enu_vel.header.stamp);
  // rclcpp::Time target_time(gga_buffer_[buffer_length-1].header.stamp);
  // rclcpp::Time initial_time(gga_buffer_[index_start].header.stamp);
  // std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
  // std::cout << "***debug-code: RtkfixPlaneValidationEstimator::estimate: " << time_now.seconds() << std::endl;
  // std::cout << "***debug-code: Target gga time:               " << target_time.seconds() <<  std::endl;
  // std::cout << "***debug-code: initial gga time:              " << initial_time.seconds() <<  std::endl;
  // std::cout << "***debug-code: index_start:                   " << index_start <<  std::endl;
  // std::cout << "***debug-code: gga_update_buffer_ init:       " << (gga_update_buffer_[index_start] ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m") << std::endl;
  // std::cout << "***debug-code: distance interval:             " << distance_buffer_[buffer_length-1] - distance_buffer_[index_start] << std::endl;
  // std::cout << "***debug-code: residual:                      " << residual_2d <<  std::endl;
  // std::cout << "***debug-code: unreliable_status_buffer init: " << (unreliable_status_buffer_[index_start] ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m") << std::endl;
  // std::cout << "***debug-code: reliable_status_buffer init:   " << (reliable_status_buffer_[index_start] ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m") << std::endl;
  // std::cout << "***debug-code: reliable_status_buffer last:   " << (reliable_status_buffer_[buffer_length-1] ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m") << std::endl;
  // std::cout << "***debug-code: is_estimation_reliable:        " << (status.is_estimation_reliable ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m") << std::endl;
  // std::cout <<  std::endl;

  return status;
}