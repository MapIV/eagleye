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
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Distance distance;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::Position enu_absolute_pos;
static geometry_msgs::msg::Vector3Stamped enu_vel;

struct PositionParameter position_parameter;
struct PositionStatus position_status;

rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub;

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

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.header.frame_id = "enu";
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

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  // node->declare_parameter(subscribe_rtklib_nav_topic_name, "eagleye/rtklib_nav_topic");
  // node->declare_parameter(position_parameter.estimated_distance ,"eagleye/position/estimated_distance";
  // node->declare_parameter(position_parameter.separation_distance ,"eagleye/position/separation_distance";
  // node->declare_parameter(position_parameter.estimated_velocity_threshold ,"eagleye/position/estimated_velocity_threshold";
  // node->declare_parameter(position_parameter.outlier_threshold "eagleye/position/outlier_threshold";
  // node->declare_parameter(position_parameter.estimated_enu_vel_coefficient ,"eagleye/position/estimated_enu_vel_coefficient";
  // node->declare_parameter(position_parameter.estimated_position_coefficient ,"eagleye/position/estimated_position_coefficient";
  // node->declare_parameter(position_parameter.ecef_base_pos_x ,"eagleye/position/ecef_base_pos_x";
  // node->declare_parameter(position_parameter.ecef_base_pos_y ,"eagleye/position/ecef_base_pos_y";
  // node->declare_parameter(position_parameter.ecef_base_pos_z ,"eagleye/position/ecef_base_pos_z";


  // subscribe_rtklib_nav_topic_name = node->get_parameter(subscribe_rtklib_nav_topic_name).as_string();
  // position_parameter.estimated_distance = node->get_parameter(position_parameter.estimated_distance).as_double();
  // position_parameter.separation_distance = node->get_parameter(position_parameter.separation_distance).as_double();
  // position_parameter.estimated_velocity_threshold = node->get_parameter(position_parameter.estimated_velocity_threshold).as_double();
  // position_parameter.estimated_velocity_threshold = node->get_parameter(position_parameter.estimated_velocity_threshold).as_double();
  // position_parameter.outlier_threshold = node->get_parameter(position_parameter.outlier_threshold).as_double();
  // position_parameter.estimated_enu_vel_coefficient = node->get_parameter(position_parameter.estimated_enu_vel_coefficient).as_double();
  // position_parameter.estimated_position_coefficient = node->get_parameter(position_parameter.estimated_position_coefficient).as_double();
  // position_parameter.ecef_base_pos_x = node->get_parameter(position_parameter.ecef_base_pos_x).as_double();
  // position_parameter.ecef_base_pos_y = node->get_parameter(position_parameter.ecef_base_pos_y).as_double();
  // position_parameter.ecef_base_pos_z = node->get_parameter(position_parameter.ecef_base_pos_z).as_double();

  n.getParam("eagleye/rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("eagleye/position/estimated_distance",position_parameter.estimated_distance);
  n.getParam("eagleye/position/separation_distance",position_parameter.separation_distance);
  n.getParam("eagleye/position/estimated_velocity_threshold",position_parameter.estimated_velocity_threshold);
  n.getParam("eagleye/position/outlier_threshold",position_parameter.outlier_threshold);
  n.getParam("eagleye/position/estimated_enu_vel_coefficient",position_parameter.estimated_enu_vel_coefficient);
  n.getParam("eagleye/position/estimated_position_coefficient",position_parameter.estimated_position_coefficient);
  n.getParam("eagleye/position/ecef_base_pos_x",position_parameter.ecef_base_pos_x);
  n.getParam("eagleye/position/ecef_base_pos_y",position_parameter.ecef_base_pos_y);
  n.getParam("eagleye/position/ecef_base_pos_z",position_parameter.ecef_base_pos_z);

  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "estimated_distance "<<position_parameter.estimated_distance<<std::endl;
  std::cout<< "separation_distance "<<position_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<position_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<position_parameter.outlier_threshold<<std::endl;
  std::cout<< "estimated_enu_vel_coefficient "<<position_parameter.estimated_enu_vel_coefficient<<std::endl;
  std::cout<< "estimated_position_coefficient "<<position_parameter.estimated_position_coefficient<<std::endl;

  auto sub1 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("eagleye/enu_vel", 1000, enu_vel_callback); //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback); //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", 1000, velocity_scale_factor_callback); //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Distance>("eagleye/distance", 1000, distance_callback); //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Heading>("eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback); //ros::TransportHints().tcpNoDelay()
  auto pub = node->create_publisher<eagleye_msgs::msg::Position>("eagleye/enu_absolute_pos", 1000);

  rclcpp::spin(node);

  return 0;
}
