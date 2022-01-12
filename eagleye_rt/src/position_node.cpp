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
 * position.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Distance distance;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::Position enu_absolute_pos;
static geometry_msgs::msg::Vector3Stamped enu_vel;
rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub;

struct PositionParameter position_parameter;
struct PositionStatus position_status;

rclcpp::Clock clock_(RCL_ROS_TIME);
tf2_ros::Buffer tfBuffer_(std::make_shared<rclcpp::Clock>(clock_));

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void on_timer()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform(position_parameter.tf_gnss_parent_flame, position_parameter.tf_gnss_child_flame, tf2::TimePointZero);

    position_parameter.tf_gnss_translation_x = transformStamped.transform.translation.x;
    position_parameter.tf_gnss_translation_y = transformStamped.transform.translation.y;
    position_parameter.tf_gnss_translation_z = transformStamped.transform.translation.z;
    position_parameter.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
    position_parameter.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
    position_parameter.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
    position_parameter.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
  }
  catch (tf2::TransformException& ex)
  {
    // RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }
}

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.header.frame_id = "base_link";
  position_estimate(rtklib_nav, velocity_scale_factor, distance, heading_interpolate_3rd, enu_vel, position_parameter, &position_status, &enu_absolute_pos);
  if(enu_absolute_pos.status.estimate_status == true)
  {
    pub->publish(enu_absolute_pos);
  }
  enu_absolute_pos.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("position");

  // tfBuffer_(node->get_clock());

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  node->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->declare_parameter("position.estimated_distance",position_parameter.estimated_distance);
  node->declare_parameter("position.separation_distance",position_parameter.separation_distance);
  node->declare_parameter("position.estimated_velocity_threshold",position_parameter.estimated_velocity_threshold);
  node->declare_parameter("position.outlier_threshold",position_parameter.outlier_threshold);
  node->declare_parameter("position.estimated_enu_vel_coefficient",position_parameter.estimated_enu_vel_coefficient);
  node->declare_parameter("position.estimated_position_coefficient",position_parameter.estimated_position_coefficient);
  node->declare_parameter("ecef_base_pos.x",position_parameter.ecef_base_pos_x);
  node->declare_parameter("ecef_base_pos.y",position_parameter.ecef_base_pos_y);
  node->declare_parameter("ecef_base_pos.z",position_parameter.ecef_base_pos_z);
  node->declare_parameter("tf_gnss_flame.parent", position_parameter.tf_gnss_parent_flame);
  node->declare_parameter("tf_gnss_flame.child", position_parameter.tf_gnss_child_flame);

  node->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->get_parameter("position.estimated_distance",position_parameter.estimated_distance);
  node->get_parameter("position.separation_distance",position_parameter.separation_distance);
  node->get_parameter("position.estimated_velocity_threshold",position_parameter.estimated_velocity_threshold);
  node->get_parameter("position.outlier_threshold",position_parameter.outlier_threshold);
  node->get_parameter("position.estimated_enu_vel_coefficient",position_parameter.estimated_enu_vel_coefficient);
  node->get_parameter("position.estimated_position_coefficient",position_parameter.estimated_position_coefficient);
  node->get_parameter("ecef_base_pos.x",position_parameter.ecef_base_pos_x);
  node->get_parameter("ecef_base_pos.y",position_parameter.ecef_base_pos_y);
  node->get_parameter("ecef_base_pos.z",position_parameter.ecef_base_pos_z);
  node->get_parameter("tf_gnss_flame.parent", position_parameter.tf_gnss_parent_flame);
  node->get_parameter("tf_gnss_flame.child", position_parameter.tf_gnss_child_flame);

  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "estimated_distance "<<position_parameter.estimated_distance<<std::endl;
  std::cout<< "separation_distance "<<position_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<position_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<position_parameter.outlier_threshold<<std::endl;
  std::cout<< "estimated_enu_vel_coefficient "<<position_parameter.estimated_enu_vel_coefficient<<std::endl;
  std::cout<< "estimated_position_coefficient "<<position_parameter.estimated_position_coefficient<<std::endl;
  std::cout<< "tf_gnss_flame.parent "<<position_parameter.tf_gnss_parent_flame<<std::endl;
  std::cout<< "tf_gnss_flame.child "<<position_parameter.tf_gnss_child_flame<<std::endl;

  auto sub1 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000, enu_vel_callback);
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub3 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", 1000, velocity_scale_factor_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", 1000, distance_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);
  
  pub = node->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos", 1000);

  const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5));
  // auto timer_callback = std::bind(&on_timer, node);
  auto timer_callback = std::bind(on_timer);
  auto timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    node->get_clock(), period_ns, std::move(timer_callback),
    node->get_node_base_interface()->get_context());
  node->get_node_timers_interface()->add_timer(timer, nullptr);

  rclcpp::spin(node);

  return 0;
}
