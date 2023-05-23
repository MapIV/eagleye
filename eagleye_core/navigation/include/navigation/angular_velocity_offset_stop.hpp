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

#ifndef ANGULAR_VELOCITY_OFFSET_STOP_HPP
#define ANGULAR_VELOCITY_OFFSET_STOP_HPP

#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include <eagleye_msgs/AngularVelocityOffset.h>

struct AngularVelocityOffsetStopParameter
{
  size_t buffer_size;
  double velocity_stop_judgement_threshold;
  double angular_stop_judgement_threshold;

  void load(const std::string& yaml_file)
  {
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      double imu_rate = conf["common"]["imu_rate"].as<double>();
      double estimated_interval = conf["angular_velocity_offset_stop"]["estimated_interval"].as<double>();
      this->buffer_size = imu_rate * estimated_interval;
      this->velocity_stop_judgement_threshold = conf["common"]["stop_judgement_threshold"].as<double>();
      this->angular_stop_judgement_threshold = conf["angular_velocity_offset_stop"]["outlier_threshold"].as<double>();
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mAngularVelocityOffsetStopParameter YAML Error: " << e.msg << "\033[m" << std::endl;
      exit(3);
    }
  }
};

struct AngularVelocityOffsetStopStatus
{
  bool is_estimated_now;
  bool is_estimation_started;
  Eigen::Vector3d offset_stop;
};

class AngularVelocityOffsetStopEstimator
{
public:
  AngularVelocityOffsetStopEstimator();
  void setParameter(const AngularVelocityOffsetStopParameter& param);
  void velocityCallback(const Eigen::Vector3d& velocity);
  AngularVelocityOffsetStopStatus imuCallback(const Eigen::Vector3d& angular_velocity);

private:
  // Param
  AngularVelocityOffsetStopParameter param_;
  // Flags
  bool is_estimation_started_;
  bool is_velocity_ready_;

  // Angular velocity
  Eigen::Vector3d estimated_offset_stop_;
  std::deque<Eigen::Vector3d> angular_velocity_buffer_;
  size_t buffer_size_;
  double angular_stop_judgement_threshold_;

  // Velocity
  Eigen::Vector3d reserved_velocity_;
  double velocity_stop_judgement_threshold_;
};

#endif /* ANGULAR_VELOCITY_OFFSET_STOP_HPP */
