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

#include <numeric>

AngularVelocityOffsetStopEstimator::AngularVelocityOffsetStopEstimator()
{
  // Initialization
  is_estimation_started_ = false;
  is_velocity_ready_ = false;
  estimated_offset_stop_ = Eigen::Vector3d::Zero();
}

void AngularVelocityOffsetStopEstimator::setParameter(const AngularVelocityOffsetStopParameter& param)
{
  // Param
  param_ = param;
}

void AngularVelocityOffsetStopEstimator::velocityCallback(const Eigen::Vector3d& velocity)
{
  // Reserve velocity
  reserved_velocity_ = velocity;
  is_velocity_ready_ = true;
}

AngularVelocityOffsetStopStatus AngularVelocityOffsetStopEstimator::imuCallback(const Eigen::Vector3d& angular_velocity)
{
  AngularVelocityOffsetStopStatus status;
  status.is_estimation_started = false;
  status.is_estimated_now = false;

  // Skip until velocity is ready
  if (!is_velocity_ready_)
    return status;

  // Bias correction
  Eigen::Vector3d unbiased_angular_velocity = angular_velocity - estimated_offset_stop_;

  // Judge stop or moving
  bool not_translating = (reserved_velocity_[0] < param_.velocity_stop_judgement_threshold);
  bool not_rotating = (std::abs(unbiased_angular_velocity[0]) < param_.angular_stop_judgement_threshold &&
                       std::abs(unbiased_angular_velocity[1]) < param_.angular_stop_judgement_threshold &&
                       std::abs(unbiased_angular_velocity[2]) < param_.angular_stop_judgement_threshold);

  if (not_translating && (!is_estimation_started_ || not_rotating))
  {
    angular_velocity_buffer_.push_back(angular_velocity);

    // Remove element if buffer size is exceeded
    if (angular_velocity_buffer_.size() > buffer_size_)
    {
      angular_velocity_buffer_.pop_front();
    }

    // Estimate offset stop if buffer is full
    if (angular_velocity_buffer_.size() == buffer_size_)
    {
      Eigen::Vector3d sum = std::accumulate(angular_velocity_buffer_.begin(), angular_velocity_buffer_.end(), Eigen::Vector3d(0.0, 0.0, 0.0));
      estimated_offset_stop_ = - sum / static_cast<double>(buffer_size_);
      is_estimation_started_ = true;
      status.is_estimated_now = true;
    }
  }
  else
  {
    angular_velocity_buffer_.clear();
  }

  status.is_estimation_started = is_estimation_started_;
  status.offset_stop = estimated_offset_stop_;

  return status;
}
