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
 * smoothing.cpp
 * Author MapIV Takanose
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static eagleye_msgs::msg::Position enu_absolute_pos,gnss_smooth_pos_enu;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub;

struct SmoothingParameter smoothing_parameter;
struct SmoothingStatus smoothing_status;

static bool use_canless_mode;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstPtr msg)
{
  velocity_status = *msg;
}

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  if(use_canless_mode && !velocity_status.status.enabled_status) return;

  rtklib_nav = *msg;
  gnss_smooth_pos_enu.header = msg->header;
  gnss_smooth_pos_enu.header.frame_id = "base_link";
  smoothing_estimate(rtklib_nav,velocity,smoothing_parameter,&smoothing_status,&gnss_smooth_pos_enu);
  pub->publish(gnss_smooth_pos_enu);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("smoothing");

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  node->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->declare_parameter("ecef_base_pos_x",smoothing_parameter.ecef_base_pos_x);
  node->declare_parameter("ecef_base_pos_y",smoothing_parameter.ecef_base_pos_y);
  node->declare_parameter("ecef_base_pos_z",smoothing_parameter.ecef_base_pos_z);
  node->declare_parameter("smoothing.estimated_number_max",smoothing_parameter.estimated_number_max);
  node->declare_parameter("smoothing.estimated_velocity_threshold",smoothing_parameter.estimated_velocity_threshold);
  node->declare_parameter("smoothing.estimated_threshold",smoothing_parameter.estimated_threshold);
  node->declare_parameter("use_canless_mode",use_canless_mode);

  node->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->get_parameter("ecef_base_pos_x",smoothing_parameter.ecef_base_pos_x);
  node->get_parameter("ecef_base_pos_y",smoothing_parameter.ecef_base_pos_y);
  node->get_parameter("ecef_base_pos_z",smoothing_parameter.ecef_base_pos_z);
  node->get_parameter("smoothing.estimated_number_max",smoothing_parameter.estimated_number_max);
  node->get_parameter("smoothing.estimated_velocity_threshold",smoothing_parameter.estimated_velocity_threshold);
  node->get_parameter("smoothing.estimated_threshold",smoothing_parameter.estimated_threshold);
  node->get_parameter("use_canless_mode",use_canless_mode);

  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "ecef_base_pos_x "<<smoothing_parameter.ecef_base_pos_x<<std::endl;
  std::cout<< "ecef_base_pos_y "<<smoothing_parameter.ecef_base_pos_y<<std::endl;
  std::cout<< "ecef_base_pos_z "<<smoothing_parameter.ecef_base_pos_z<<std::endl;
  std::cout<< "estimated_number_max "<<smoothing_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<smoothing_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_threshold "<<smoothing_parameter.estimated_threshold<<std::endl;

  auto sub1 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub2 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub3 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  pub = node->create_publisher<eagleye_msgs::msg::Position>("gnss_smooth_pos_enu", rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
