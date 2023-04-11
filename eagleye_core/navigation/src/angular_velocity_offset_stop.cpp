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
 * angular_velocity_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "navigation/angular_velocity_offset_stop.hpp"

void angular_velocity_offset_stop_estimate(const geometry_msgs::TwistStamped velocity, const sensor_msgs::Imu imu,
  const AngularVelocityOffsetStopParameter angular_velocity_stop_parameter, AngularVelocityOffsetStopStatus* angular_velocity_stop_status,
  eagleye_msgs::AngularVelocityOffset* angular_velocity_offset_stop)
{
  size_t buffer_size = angular_velocity_stop_parameter.imu_rate * angular_velocity_stop_paramter.estimated_interval * 2;
  bool estimate_now = false;

  // Push angular velocity while stopping
  if (velocity.twist.linear.x < angular_velocity_stop_parameter.stop_judgment_threshold)
  {
    Eigen::Vector3d angular_velocity(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
    angular_velocity_stop_status->buffer.push_back(angular_velocity);

    if (angular_velocity_stop_status->buffer.size() > buffer_size)
    {
      angular_velocity_stop_status->buffer.pop_front();
    }

    if (angular_velocity_stop_status->buffer.size() == buffer_size)
    {
      Eigen::Vector3d sum = std::accumlate(angular_velocity_offset_stop_status.buffer.begin(),
                                           angular_velocity_offset_stop_status.buffer.end(),
                                           Eigen::Vector3d(0.0, 0.0, 0.0));
      angular_velocity_stop_status->offset_stop = - sum / static_cast<double>(buffer_size);
      angular_velocity_stop_status->estimate_start_status = true;
      estimate_now = true;
    }
  }
  else
  {
    angular_velocity_stop_status->buffer.clear();
  }

  angular_velocity_offset_stop->angular_velocity_offset.x = angular_velocity_stop_status->offset_stop[0];
  angular_velocity_offset_stop->angular_velocity_offset.y = angular_velocity_stop_status->offset_stop[1];
  angular_velocity_offset_stop->angular_velocity_offset.z = angular_velocity_stop_status->offset_stop[2];
  angular_velocity_offset_stop->status.estimate_status = estimate_now;
  angular_velocity_offset_stop->status.enable_status = angular_velocity_stop_status->estimate_start_status;

  // if (angular_velocity_stop_status->estimate_start_status == false)
  // {
  //   angular_velocity_stop_status->rollrate_buffer.push_back(imu.angular_velocity.x);
  //   angular_velocity_stop_status->pitchrate_buffer.push_back(imu.angular_velocity.y);
  //   angular_velocity_stop_status->yawrate_buffer.push_back(imu.angular_velocity.z);
  // }
  // else if ( std::fabs(std::fabs(angular_velocity_stop_status->yawrate_offset_stop_last) - std::fabs(imu.angular_velocity.z)) <
  //   angular_velocity_stop_parameter.outlier_threshold && angular_velocity_stop_status->estimate_start_status == true)
  //   // if があるので==trueの条件はいらない
  //   // この条件の気持ちが分からない
  // {
  //   angular_velocity_stop_status->rollrate_buffer.push_back(imu.angular_velocity.x);
  //   angular_velocity_stop_status->pitchrate_buffer.push_back(imu.angular_velocity.y);
  //   angular_velocity_stop_status->yawrate_buffer.push_back(imu.angular_velocity.z);
  // }

  // rollrate_buffer_length = std::distance(angular_velocity_stop_status->rollrate_buffer.begin(), angular_velocity_stop_status->rollrate_buffer.end());
  // pitchrate_buffer_length = std::distance(angular_velocity_stop_status->pitchrate_buffer.begin(), angular_velocity_stop_status->pitchrate_buffer.end());
  // yawrate_buffer_length = std::distance(angular_velocity_stop_status->yawrate_buffer.begin(), angular_velocity_stop_status->yawrate_buffer.end());
  // // .size()でいい
  // // 個別で見る必要ある？

  // if (yawrate_buffer_length > estimated_buffer_number + estimated_time_buffer_number)
  // // estimated_buffer_number == estimated_time_buffer_numberじゃない？
  // {
  //   angular_velocity_stop_status->rollrate_buffer.erase(angular_velocity_stop_status->rollrate_buffer.begin());
  //   angular_velocity_stop_status->pitchrate_buffer.erase(angular_velocity_stop_status->pitchrate_buffer.begin());
  //   angular_velocity_stop_status->yawrate_buffer.erase(angular_velocity_stop_status->yawrate_buffer.begin());
  //   // dequeかring bufferが良さそう？
  // }

  // if (velocity.twist.linear.x < angular_velocity_stop_parameter.stop_judgment_threshold)
  // {
  //   ++angular_velocity_stop_status->stop_count;
  // }
  // else
  // {
  //   angular_velocity_stop_status->stop_count = 0;
  // }

  // // mean
  // if (angular_velocity_stop_status->stop_count > estimated_buffer_number + estimated_time_buffer_number)
  // {
  //   roll_tmp = 0.0;
  //   pitch_tmp = 0.0;
  //   yaw_tmp = 0.0;
  //   for (i = 0; i < estimated_buffer_number; i++)
  //   {
  //     roll_tmp += angular_velocity_stop_status->rollrate_buffer[i];
  //     pitch_tmp += angular_velocity_stop_status->pitchrate_buffer[i];
  //     yaw_tmp += angular_velocity_stop_status->yawrate_buffer[i];
  //   }
  //   angular_velocity_offset_stop->angular_velocity_offset.x = -1 * roll_tmp / estimated_buffer_number;
  //   angular_velocity_offset_stop->angular_velocity_offset.y = -1 * pitch_tmp / estimated_buffer_number;
  //   angular_velocity_offset_stop->angular_velocity_offset.z = -1 * yaw_tmp / estimated_buffer_number;
  //   // accumlateとか使いたい
  //   // ここでangular_velocity_stop_statusに代入すべき
  //   // 内部で計算してその結果をメッセージに反映する順序が正しい
  //   angular_velocity_offset_stop->status.enabled_status = true;
  //   angular_velocity_offset_stop->status.estimate_status = true;
  //   angular_velocity_stop_status->estimate_start_status = true;
  //   // flag多すぎ問題
  // }
  // else
  // {
  //   // ここのelseはいらなくなる
  //   angular_velocity_offset_stop->angular_velocity_offset.x = angular_velocity_stop_status->rollrate_offset_stop_last;
  //   angular_velocity_offset_stop->angular_velocity_offset.y = angular_velocity_stop_status->pitchrate_offset_stop_last;
  //   angular_velocity_offset_stop->angular_velocity_offset.z = angular_velocity_stop_status->yawrate_offset_stop_last;
  //   angular_velocity_offset_stop->status.estimate_status = false;
  //   // 左はstructなのに右は別々の変数なのは変
  // }
  // if (angular_velocity_stop_status->estimate_start_status == false)
  // {
  //   angular_velocity_offset_stop->angular_velocity_offset.x = initial_angular_velocity_offset_stop;
  //   angular_velocity_offset_stop->angular_velocity_offset.y = initial_angular_velocity_offset_stop;
  //   angular_velocity_offset_stop->angular_velocity_offset.z = initial_angular_velocity_offset_stop;
  //   // 0.0でも良さそう
  //   angular_velocity_offset_stop->status.estimate_status = false;
  //   angular_velocity_offset_stop->status.enabled_status = false;
  // }
  // angular_velocity_stop_status->rollrate_offset_stop_last = angular_velocity_offset_stop->angular_velocity_offset.x;
  // angular_velocity_stop_status->pitchrate_offset_stop_last = angular_velocity_offset_stop->angular_velocity_offset.y;
  // angular_velocity_stop_status->yawrate_offset_stop_last = angular_velocity_offset_stop->angular_velocity_offset.z;
  // // これもいらなくなる
}
