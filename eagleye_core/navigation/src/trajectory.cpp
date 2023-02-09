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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

void calculate_covariance(const geometry_msgs::msg::TwistStamped velocity, const eagleye_msgs::msg::StatusStamped velocity_status,
  const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop, const TrajectoryParameter trajectory_parameter,
  geometry_msgs::msg::TwistWithCovarianceStamped* eagleye_twist_with_covariance)
{
  double noise_velocity;
  double noise_yaw_rate;

  if(velocity_status.status.enabled_status)
  {
    noise_velocity = trajectory_parameter.sensor_noise_velocity * trajectory_parameter.sensor_noise_velocity;
  }
  else
  {
    noise_velocity = trajectory_parameter.sensor_noise_velocity * trajectory_parameter.sensor_noise_velocity
      + (velocity.twist.linear.x*trajectory_parameter.sensor_scale_noise_velocity)*(velocity.twist.linear.x*trajectory_parameter.sensor_scale_noise_velocity);
  }

  if(yaw_rate_offset_stop.status.enabled_status)
  {
    noise_yaw_rate = trajectory_parameter.sensor_noise_yaw_rate * trajectory_parameter.sensor_noise_yaw_rate;
  }
  else
  {
    noise_yaw_rate = trajectory_parameter.sensor_noise_yaw_rate * trajectory_parameter.sensor_noise_yaw_rate
      + trajectory_parameter.sensor_bias_noise_yaw_rate * trajectory_parameter.sensor_bias_noise_yaw_rate;
  }

  eagleye_twist_with_covariance->twist.covariance[0] = noise_velocity;
  eagleye_twist_with_covariance->twist.covariance[35] = noise_yaw_rate;
}

void trajectory_estimate(const sensor_msgs::msg::Imu imu,  const geometry_msgs::msg::TwistStamped velocity, 
  const eagleye_msgs::msg::StatusStamped velocity_status, const eagleye_msgs::msg::Heading heading_interpolate_3rd,
  const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop, const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd,
  const TrajectoryParameter trajectory_parameter, TrajectoryStatus* trajectory_status, geometry_msgs::msg::Vector3Stamped* enu_vel,
  eagleye_msgs::msg::Position* enu_relative_pos, geometry_msgs::msg::TwistStamped* eagleye_twist,
  geometry_msgs::msg::TwistWithCovarianceStamped* eagleye_twist_with_covariance)
{
  rclcpp::Time ros_clock(imu.header.stamp);
  auto imu_time = ros_clock.seconds();

  if (std::abs(velocity.twist.linear.x) > trajectory_parameter.stop_judgment_threshold && yaw_rate_offset_2nd.status.enabled_status == true)
  {
    eagleye_twist->twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset); //Inverted because the coordinate system is reversed
    eagleye_twist_with_covariance->twist.twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset); //Inverted because the coordinate system is reversed
  }
  else
  {
    eagleye_twist->twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_stop.yaw_rate_offset); //Inverted because the coordinate system is reversed
    eagleye_twist_with_covariance->twist.twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_stop.yaw_rate_offset); //Inverted because the coordinate system is reversed
  }

  eagleye_twist->twist.linear.x = velocity.twist.linear.x;
  eagleye_twist_with_covariance->twist.twist.linear.x = velocity.twist.linear.x;

  calculate_covariance(velocity, velocity_status, yaw_rate_offset_stop, trajectory_parameter, eagleye_twist_with_covariance);

  if (trajectory_status->estimate_status_count == 0 && velocity_status.status.enabled_status == true && heading_interpolate_3rd.status.enabled_status == true)
  {
    trajectory_status->estimate_status_count = 1;
    trajectory_status->heading_last = heading_interpolate_3rd.heading_angle;
  }
  else if (trajectory_status->estimate_status_count == 1)
  {
    trajectory_status->estimate_status_count = 2;
  }

  if (trajectory_status->estimate_status_count == 2)
  {
    enu_vel->vector.x = sin(heading_interpolate_3rd.heading_angle) * velocity.twist.linear.x; //vel_e
    enu_vel->vector.y = cos(heading_interpolate_3rd.heading_angle) * velocity.twist.linear.x; //vel_n
    enu_vel->vector.z = 0; //vel_u
  }

  if (trajectory_status->estimate_status_count == 2 && std::abs(velocity.twist.linear.x) > 0 && trajectory_status->time_last != 0)
  {
    if(std::abs(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) < trajectory_parameter.curve_judgment_threshold)
    {
      enu_relative_pos->enu_pos.x = enu_relative_pos->enu_pos.x + enu_vel->vector.x * (imu_time - trajectory_status->time_last);
      enu_relative_pos->enu_pos.y = enu_relative_pos->enu_pos.y + enu_vel->vector.y * (imu_time - trajectory_status->time_last);
      enu_relative_pos->enu_pos.z = 0;
    }
    else if((imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) != 0)
    {
      enu_relative_pos->enu_pos.x = enu_relative_pos->enu_pos.x + velocity.twist.linear.x/(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) * ( -cos(trajectory_status->heading_last+(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset)*(imu_time - trajectory_status->time_last)) + cos(trajectory_status->heading_last));
      enu_relative_pos->enu_pos.y = enu_relative_pos->enu_pos.y + velocity.twist.linear.x/(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) * ( sin(trajectory_status->heading_last+(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset)*(imu_time - trajectory_status->time_last)) - sin(trajectory_status->heading_last));
      enu_relative_pos->enu_pos.z = 0;
    }
    else{
      enu_relative_pos->enu_pos.x = enu_relative_pos->enu_pos.x + enu_vel->vector.x * (imu_time - trajectory_status->time_last);
      enu_relative_pos->enu_pos.y = enu_relative_pos->enu_pos.y + enu_vel->vector.y * (imu_time - trajectory_status->time_last);
      enu_relative_pos->enu_pos.z = 0;
    }

    enu_relative_pos->status.enabled_status = enu_relative_pos->status.estimate_status = true;
  }

  trajectory_status->heading_last = heading_interpolate_3rd.heading_angle;
  trajectory_status->time_last = imu_time;
}

void trajectory3d_estimate(const sensor_msgs::msg::Imu imu, const geometry_msgs::msg::TwistStamped velocity, 
  const eagleye_msgs::msg::StatusStamped velocity_status, const eagleye_msgs::msg::Heading heading_interpolate_3rd,
  const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop, const eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd,
  const eagleye_msgs::msg::Pitching pitching, const TrajectoryParameter trajectory_parameter, TrajectoryStatus* trajectory_status,
  geometry_msgs::msg::Vector3Stamped* enu_vel, eagleye_msgs::msg::Position* enu_relative_pos, geometry_msgs::msg::TwistStamped* eagleye_twist,
  geometry_msgs::msg::TwistWithCovarianceStamped* eagleye_twist_with_covariance)
{
  rclcpp::Time ros_clock(imu.header.stamp);
  auto imu_time = ros_clock.seconds();

  if (std::abs(velocity.twist.linear.x) > trajectory_parameter.stop_judgment_threshold && yaw_rate_offset_2nd.status.enabled_status == true)
  {
    eagleye_twist->twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset); //Inverted because the coordinate system is reversed
    eagleye_twist_with_covariance->twist.twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset); //Inverted because the coordinate system is reversed
  }
  else
  {
    eagleye_twist->twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_stop.yaw_rate_offset); //Inverted because the coordinate system is reversed
    eagleye_twist_with_covariance->twist.twist.angular.z = -1 * (imu.angular_velocity.z + yaw_rate_offset_stop.yaw_rate_offset); //Inverted because the coordinate system is reversed
  }

  eagleye_twist->twist.linear.x = velocity.twist.linear.x;
  eagleye_twist_with_covariance->twist.twist.linear.x = velocity.twist.linear.x;

  calculate_covariance(velocity, velocity_status, yaw_rate_offset_stop, trajectory_parameter, eagleye_twist_with_covariance);

  if (trajectory_status->estimate_status_count == 0 && velocity_status.status.enabled_status == true && heading_interpolate_3rd.status.enabled_status == true)
  {
    trajectory_status->estimate_status_count = 1;
    trajectory_status->heading_last = heading_interpolate_3rd.heading_angle;
  }
  else if (trajectory_status->estimate_status_count == 1)
  {
    trajectory_status->estimate_status_count = 2;
  }

  if (trajectory_status->estimate_status_count == 2)
  {
    enu_vel->vector.x = sin(heading_interpolate_3rd.heading_angle) * cos(pitching.pitching_angle) * velocity.twist.linear.x; //vel_e
    enu_vel->vector.y = cos(heading_interpolate_3rd.heading_angle) * cos(pitching.pitching_angle) * velocity.twist.linear.x; //vel_n
    enu_vel->vector.z = sin(pitching.pitching_angle) * velocity.twist.linear.x; //vel_u
  }

  if (trajectory_status->estimate_status_count == 2 && std::abs(velocity.twist.linear.x) > 0 && trajectory_status->time_last != 0)
  {
    if(std::abs(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) < trajectory_parameter.curve_judgment_threshold)
    {
      enu_relative_pos->enu_pos.x = enu_relative_pos->enu_pos.x + enu_vel->vector.x * (imu_time - trajectory_status->time_last);
      enu_relative_pos->enu_pos.y = enu_relative_pos->enu_pos.y + enu_vel->vector.y * (imu_time - trajectory_status->time_last);
      enu_relative_pos->enu_pos.z = enu_relative_pos->enu_pos.z + enu_vel->vector.z * (imu_time - trajectory_status->time_last);
    }
    else
    {
      enu_relative_pos->enu_pos.x = enu_relative_pos->enu_pos.x + velocity.twist.linear.x/(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) * ( -cos(trajectory_status->heading_last+(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset)*(imu_time - trajectory_status->time_last)) + cos(trajectory_status->heading_last));
      enu_relative_pos->enu_pos.y = enu_relative_pos->enu_pos.y + velocity.twist.linear.x/(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset) * ( sin(trajectory_status->heading_last+(imu.angular_velocity.z + yaw_rate_offset_2nd.yaw_rate_offset)*(imu_time - trajectory_status->time_last)) - sin(trajectory_status->heading_last));
      enu_relative_pos->enu_pos.z = enu_relative_pos->enu_pos.z + enu_vel->vector.z * (imu_time - trajectory_status->time_last);
    }

    enu_relative_pos->status.enabled_status = enu_relative_pos->status.estimate_status = true;
  }

  trajectory_status->heading_last = heading_interpolate_3rd.heading_angle;
  trajectory_status->time_last = imu_time;
}
