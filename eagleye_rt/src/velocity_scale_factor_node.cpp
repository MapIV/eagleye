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
 * velocity_scale_factor.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static geometry_msgs::msg::TwistStamped velocity;
static sensor_msgs::msg::Imu imu;


rclcpp::Publisher<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr pub;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;

struct VelocityScaleFactorParameter velocity_scale_factor_parameter;
struct VelocityScaleFactorStatus velocity_scale_factor_status;

bool is_first_move = false;

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;

  if(is_first_move == false && msg->twist.linear.x > velocity_scale_factor_parameter.estimated_velocity_threshold)
  {
    is_first_move = true;
  }
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(is_first_move == false)
  {
    return;
  }

  imu = *msg;
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.header.frame_id = "base_link";
  velocity_scale_factor_estimate(rtklib_nav,velocity,velocity_scale_factor_parameter,&velocity_scale_factor_status,&velocity_scale_factor);
  pub->publish(velocity_scale_factor);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("velocity_scale_factor");

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  node->declare_parameter("twist_topic",subscribe_twist_topic_name);
  node->declare_parameter("imu_topic",subscribe_imu_topic_name);
  node->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->declare_parameter("velocity_scale_factor.estimated_number_min",velocity_scale_factor_parameter.estimated_number_min);
  node->declare_parameter("velocity_scale_factor.estimated_number_max",velocity_scale_factor_parameter.estimated_number_max);
  node->declare_parameter("velocity_scale_factor.estimated_velocity_threshold",velocity_scale_factor_parameter.estimated_velocity_threshold);
  node->declare_parameter("velocity_scale_factor.estimated_coefficient",velocity_scale_factor_parameter.estimated_coefficient);

  node->get_parameter("twist_topic",subscribe_twist_topic_name);
  node->get_parameter("imu_topic",subscribe_imu_topic_name);
  node->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->get_parameter("velocity_scale_factor.estimated_number_min",velocity_scale_factor_parameter.estimated_number_min);
  node->get_parameter("velocity_scale_factor.estimated_number_max",velocity_scale_factor_parameter.estimated_number_max);
  node->get_parameter("velocity_scale_factor.estimated_velocity_threshold",velocity_scale_factor_parameter.estimated_velocity_threshold);
  node->get_parameter("velocity_scale_factor.estimated_coefficient",velocity_scale_factor_parameter.estimated_coefficient);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "estimated_number_min "<<velocity_scale_factor_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<velocity_scale_factor_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<velocity_scale_factor_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_coefficient "<<velocity_scale_factor_parameter.estimated_coefficient<<std::endl;

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback);
  auto sub2 = node->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, velocity_callback);
  auto sub3 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  pub = node->create_publisher<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
