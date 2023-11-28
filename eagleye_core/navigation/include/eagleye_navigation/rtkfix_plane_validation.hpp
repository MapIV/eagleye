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

#ifndef RTKFIX_PLANE_VALIDATION_HPP
#define RTKFIX_PLANE_VALIDATION_HPP

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <eagleye_msgs/msg/distance.hpp>

#include <rclcpp/rclcpp.hpp>

// RtkfixPlaneValidation
struct RtkfixPlaneValidationParameter
{
  double validation_minimum_interval;
  double validation_maximum_interval;
  double outlier_threshold;

  void load(const std::string& yaml_file)
  {
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      this->validation_minimum_interval = conf["/**"]["ros__parameters"]["rtkfix_plane_validation"]["validation_minimum_interval"].as<double>();
      this->validation_maximum_interval = conf["/**"]["ros__parameters"]["rtkfix_plane_validation"]["validation_maximum_interval"].as<double>();
      this->outlier_threshold = conf["/**"]["ros__parameters"]["rtkfix_plane_validation"]["outlier_threshold"].as<double>();
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mRtkfixPlaneValidationParameter YAML Error: " << e.msg << "\033[m" << std::endl;
      exit(3);
    }
  }
};

struct RtkfixPlaneValidationStatus
{
  bool is_estimated_now;
  bool is_estimation_started;
  bool is_estimation_reliable;
  sensor_msgs::msg::NavSatFix fix_msg;
};

class RtkfixPlaneValidationEstimator
{
public:
  RtkfixPlaneValidationEstimator();
  void setParameter(const RtkfixPlaneValidationParameter& param);
  RtkfixPlaneValidationStatus estimate(const nmea_msgs::msg::Gpgga gga, const eagleye_msgs::msg::Distance distance, const geometry_msgs::msg::Vector3Stamped enu_vel);

private:
//   // Param
  RtkfixPlaneValidationParameter param_;

  double gga_time_last_;
  bool is_first_epoch_;

  std::vector<nmea_msgs::msg::Gpgga> gga_buffer_;
  std::vector<bool> gga_update_buffer_;
  std::vector<bool> reliable_status_buffer_;
  std::vector<bool> unreliable_status_buffer_;
  std::vector<double> distance_buffer_;
  std::vector<geometry_msgs::msg::Vector3Stamped> enu_vel_buffer_;
};

#endif /* RTKFIX_PLANE_VALIDATION_HPP */