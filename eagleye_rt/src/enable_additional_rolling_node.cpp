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
 * enable_additional_rolling_node.cpp
 * Author MapIV Hoda
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

rclcpp::Publisher<eagleye_msgs::msg::AccYOffset>::SharedPtr _pub1;
rclcpp::Publisher<eagleye_msgs::msg::Rolling>::SharedPtr _pub2;

static eagleye_msgs::msg::VelocityScaleFactor _velocity_scale_factor;
static geometry_msgs::msg::TwistStamped _velocity;
static eagleye_msgs::msg::StatusStamped _velocity_status;
static eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_2nd;
static eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_stop;
static eagleye_msgs::msg::Distance _distance;
static geometry_msgs::msg::PoseStamped _localization_pose;
static eagleye_msgs::msg::AngularVelocityOffset _angular_velocity_offset_stop;
static sensor_msgs::msg::Imu _imu;

static eagleye_msgs::msg::Rolling _rolling_angle;
static eagleye_msgs::msg::AccYOffset _acc_y_offset;

struct EnableAdditionalRollingParameter _rolling_parameter;
struct EnableAdditionalRollingStatus _rolling_status;

static bool _use_can_less_mode;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  _velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  _velocity_status = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  _velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  _distance = *msg;
}

void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  _yaw_rate_offset_2nd = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  _yaw_rate_offset_stop = *msg;
}

void localization_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  _localization_pose = *msg;
}

void angular_velocity_offset_stop_callback(const eagleye_msgs::msg::AngularVelocityOffset::ConstSharedPtr msg)
{
  _angular_velocity_offset_stop = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(_use_can_less_mode && !_velocity_status.status.enabled_status) return;

  eagleye_msgs::msg::StatusStamped velocity_enable_status;
  if(_use_can_less_mode)
  {
    velocity_enable_status = _velocity_status;
  }
  else
  {
    velocity_enable_status.header = _velocity_scale_factor.header;
    velocity_enable_status.status = _velocity_scale_factor.status;
  }

  _imu = *msg;
  _acc_y_offset.header = msg->header;
  _acc_y_offset.header.frame_id = "imu";
  _rolling_angle.header = msg->header;
  _rolling_angle.header.frame_id = "base_link";
  enable_additional_rolling_estimate(_velocity, velocity_enable_status, _yaw_rate_offset_2nd, _yaw_rate_offset_stop, _distance, _imu,
    _localization_pose, _angular_velocity_offset_stop, _rolling_parameter, &_rolling_status, &_rolling_angle, &_acc_y_offset);
  _pub1->publish(_acc_y_offset);
  _pub2->publish(_rolling_angle);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_enable_additional_rolling");

  std::string subscribe_localization_pose_topic_name;

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    subscribe_localization_pose_topic_name = conf["/**"]["ros__parameters"]["localization_pose_topic"].as<std::string>();

    _rolling_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    _rolling_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();

    _rolling_parameter.update_distance = conf["/**"]["ros__parameters"]["enable_additional_rolling"]["update_distance"].as<double>();
    _rolling_parameter.moving_average_time = conf["/**"]["ros__parameters"]["enable_additional_rolling"]["moving_average_time"].as<double>();
    _rolling_parameter.sync_judgment_threshold = conf["/**"]["ros__parameters"]["enable_additional_rolling"]["sync_judgment_threshold"].as<double>();
    _rolling_parameter.sync_search_period = conf["/**"]["ros__parameters"]["enable_additional_rolling"]["sync_search_period"].as<double>();

    std::cout<< "subscribe_localization_pose_topic_name " << subscribe_localization_pose_topic_name << std::endl;

    std::cout << "imu_rate " << _rolling_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << _rolling_parameter.stop_judgment_threshold << std::endl;

    std::cout << "update_distance " << _rolling_parameter.update_distance << std::endl;
    std::cout << "moving_average_time " << _rolling_parameter.moving_average_time << std::endl;
    std::cout << "sync_judgment_threshold " << _rolling_parameter.sync_judgment_threshold << std::endl;
    std::cout << "sync_search_period " << _rolling_parameter.sync_search_period << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31menable_additional_rolling Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  auto sub1 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", 1000, velocity_scale_factor_callback);
  auto sub2 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", 1000, yaw_rate_offset_2nd_callback);
  auto sub3 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", 1000, yaw_rate_offset_stop_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", 1000, distance_callback);
  auto sub5 = node->create_subscription<geometry_msgs::msg::PoseStamped>(subscribe_localization_pose_topic_name, 1000, localization_pose_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::AngularVelocityOffset>("angular_velocity_offset_stop", 1000, angular_velocity_offset_stop_callback);
  auto sub7 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);

  _pub1 = node->create_publisher<eagleye_msgs::msg::AccYOffset>("acc_y_offset_additional_rolling", 1000);
  _pub2 = node->create_publisher<eagleye_msgs::msg::Rolling>("enable_additional_rolling", 1000);

  rclcpp::spin(node);

  return 0;
}