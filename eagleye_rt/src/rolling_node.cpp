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

static eagleye_msgs::msg::VelocityScaleFactor _velocity_scale_factor_msg;
static eagleye_msgs::msg::YawrateOffset _yawrate_offset_2nd_msg;
static eagleye_msgs::msg::YawrateOffset _yawrate_offset_stop_msg;
static sensor_msgs::msg::Imu _imu_msg;

static eagleye_msgs::msg::Rolling _rolling_msg;

struct RollingParameter _rolling_parameter;
struct RollingStatus _rolling_status;

static std::string _subscribe_imu_topic_name;

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor_msg = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_stop_msg = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_2nd_msg = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstPtr& msg)
{
  _imu_msg = *msg;
  rolling_estimate(_imu_msg, _velocity_scale_factor_msg, _yawrate_offset_stop_msg, _yawrate_offset_2nd_msg,
                   _rolling_parameter, &_rolling_status, &_rolling_msg);
  _rolling_pub->publish(_rolling_msg);
}

void setParam(rclcpp::Node::SharedPtr &  node)
{
  node->declare_parameter("imu_topic",_subscribe_imu_topic_name);
  node->declare_parameter("reverse_imu",_rolling_parameter.reverse_imu);
  node->declare_parameter("rolling.stop_judgment_velocity_threshold",_rolling_parameter.stop_judgment_velocity_threshold);
  node->declare_parameter("rolling.filter_process_noise",_rolling_parameter.filter_process_noise);
  node->declare_parameter("rolling.filter_observation_noise",_rolling_parameter.filter_observation_noise);

  node->get_parameter("imu_topic",_subscribe_imu_topic_name);
  node->get_parameter("reverse_imu",_rolling_parameter.reverse_imu);
  node->get_parameter("rolling.stop_judgment_velocity_threshold",_rolling_parameter.stop_judgment_velocity_threshold);
  node->get_parameter("rolling.filter_process_noise",_rolling_parameter.filter_process_noise);
  node->get_parameter("rolling.filter_observation_noise",_rolling_parameter.filter_observation_noise);

  std::cout << "subscribe_imu_topic_name " << _subscribe_imu_topic_name << std::endl;
  std::cout << "reverse_imu " << _rolling_parameter.reverse_imu << std::endl;
  std::cout << "stop_judgment_velocity_threshold " << _rolling_parameter.stop_judgment_velocity_threshold << std::endl;
  std::cout << "filter_process_noise " << _rolling_parameter.filter_process_noise << std::endl;
  std::cout << "filter_observation_noise " << _rolling_parameter.filter_observation_noise << std::endl;
}

void rolling_node(rclcpp::Node::SharedPtr &  node)
{
  setParam(node);

  auto imu_sub =
      node->create_subscription<sensor_msgs::msg::Imu>(_subscribe_imu_topic_name, 1000, imu_callback);
  auto velocity_scale_factor_sub =
      node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", 1000, velocity_scale_factor_callback);
  auto yawrate_offset_2nd_sub =
      node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback);
  auto yawrate_offset_stop_sub =
      node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", 1000, yawrate_offset_stop_callback);

  _rolling_pub = node->create_publisher<eagleye_msgs::msg::Rolling>("rolling", 1000);

  rclcpp::spin(node);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rolling");

  rolling_node(node);

  return 0;
}