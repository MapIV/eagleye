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
 * position_interpolate.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static eagleye_msgs::msg::Position enu_absolute_pos;
static geometry_msgs::msg::Vector3Stamped enu_vel;
static eagleye_msgs::msg::Height height;
static eagleye_msgs::msg::Position gnss_smooth_pos;
static sensor_msgs::msg::NavSatFix fix;


static eagleye_msgs::msg::Position enu_absolute_pos_interpolate;
static sensor_msgs::msg::NavSatFix eagleye_fix;
rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub1;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub2;

struct PositionInterpolateParameter position_interpolate_parameter;
struct PositionInterpolateStatus position_interpolate_status;

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  fix.header = msg->header;
  fix.status = msg->status;
  fix.latitude = msg->latitude;
  fix.longitude = msg->longitude;
  fix.altitude = msg->altitude;
  fix.position_covariance = msg->position_covariance;
  fix.position_covariance_type = msg->position_covariance_type;
}

void enu_absolute_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}

void gnss_smooth_pos_enu_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  gnss_smooth_pos.header = msg->header;
  gnss_smooth_pos.enu_pos = msg->enu_pos;
  gnss_smooth_pos.ecef_base_pos = msg->ecef_base_pos;
  gnss_smooth_pos.status = msg->status;
}

void height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
{
  height.header = msg->header;
  height.height = msg->height;
  height.status = msg->status;
}

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  rclcpp::Time ros_clock(fix.header.stamp);
  auto fix_time = ros_clock.seconds();

  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_pos_interpolate.header = msg->header;
  enu_absolute_pos_interpolate.header.frame_id = "base_link";
  eagleye_fix.header = msg->header;
  eagleye_fix.header.frame_id = "gnss";
  position_interpolate_estimate(enu_absolute_pos,enu_vel,gnss_smooth_pos,height,position_interpolate_parameter,&position_interpolate_status,&enu_absolute_pos_interpolate,&eagleye_fix);
  if(enu_absolute_pos.status.enabled_status == true)
  {
    pub1->publish(enu_absolute_pos_interpolate);
    pub2->publish(eagleye_fix);
  }
  else if (fix_time != 0)
  {
    pub2->publish(fix);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("position_interpolate");

  std::string subscribe_navsatfix_topic_name = "/navsat/fix";

  node->declare_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->declare_parameter("position_interpolate.number_buffer_max", position_interpolate_parameter.number_buffer_max);
  node->declare_parameter("position_interpolate.stop_judgment_velocity_threshold", position_interpolate_parameter.stop_judgment_velocity_threshold);

  node->get_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->get_parameter("position_interpolate.number_buffer_max", position_interpolate_parameter.number_buffer_max);
  node->get_parameter("position_interpolate.stop_judgment_velocity_threshold", position_interpolate_parameter.stop_judgment_velocity_threshold);
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;
  std::cout<< "number_buffer_max "<<position_interpolate_parameter.number_buffer_max<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<position_interpolate_parameter.stop_judgment_velocity_threshold<<std::endl;


  auto sub1 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", rclcpp::QoS(10), enu_vel_callback); //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<eagleye_msgs::msg::Position>("enu_absolute_pos", rclcpp::QoS(10), enu_absolute_pos_callback); //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::Position>("gnss_smooth_pos_enu", rclcpp::QoS(10), gnss_smooth_pos_enu_callback); //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Height>("height", rclcpp::QoS(10), height_callback); //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_navsatfix_topic_name, 1000, fix_callback); //ros::TransportHints().tcpNoDelay()
  pub1 = node->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos_interpolate", 1000);
  pub2 = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1000);

  rclcpp::spin(node);

  return 0;
}
