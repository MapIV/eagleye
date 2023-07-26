// Copyright (c) 2022, Map IV, Inc.
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
 * rolling_node.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

rclcpp::Publisher<eagleye_msgs::msg::Rolling>::SharedPtr _rolling_pub;

static geometry_msgs::msg::TwistStamped _velocity_msg;
static eagleye_msgs::msg::StatusStamped _velocity_status_msg;
static eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_2nd_msg;
static eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_stop_msg;
static sensor_msgs::msg::Imu _imu_msg;

static eagleye_msgs::msg::Rolling _rolling_msg;

struct RollingParameter _rolling_parameter;
struct RollingStatus _rolling_status;

static bool _use_can_less_mode;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  _velocity_msg = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  _velocity_status_msg = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  _yaw_rate_offset_stop_msg = *msg;
}

void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  _yaw_rate_offset_2nd_msg = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(_use_can_less_mode && !_velocity_status_msg.status.enabled_status) return;
  _imu_msg = *msg;
  rolling_estimate(_imu_msg, _velocity_msg, _yaw_rate_offset_stop_msg, _yaw_rate_offset_2nd_msg,
                   _rolling_parameter, &_rolling_status, &_rolling_msg);
  _rolling_pub->publish(_rolling_msg);
}

void setParam(rclcpp::Node::SharedPtr node)
{
  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
    _rolling_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    _rolling_parameter.filter_process_noise = conf["/**"]["ros__parameters"]["rolling"]["filter_process_noise"].as<double>();
    _rolling_parameter.filter_observation_noise = conf["/**"]["ros__parameters"]["rolling"]["filter_observation_noise"].as<double>();

    std::cout<< "use_can_less_mode " << _use_can_less_mode << std::endl;
    std::cout << "stop_judgment_threshold " << _rolling_parameter.stop_judgment_threshold << std::endl;
    std::cout << "filter_process_noise " << _rolling_parameter.filter_process_noise << std::endl;
    std::cout << "filter_observation_noise " << _rolling_parameter.filter_observation_noise << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mrolling Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
}

void rolling_node(rclcpp::Node::SharedPtr node)
{
  setParam(node);

  auto imu_sub =
      node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto velocity_sub =
      node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto velocity_status_sub =
      node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto yaw_rate_offset_2nd_sub =
      node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", 1000, yaw_rate_offset_2nd_callback);
  auto yaw_rate_offset_stop_sub =
      node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", 1000, yaw_rate_offset_stop_callback);

  _rolling_pub = node->create_publisher<eagleye_msgs::msg::Rolling>("rolling", 1000);

  rclcpp::spin(node);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_rolling");

  rolling_node(node);

  return 0;
}
