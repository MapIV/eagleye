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
 * rtk_deadreckoning_node.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static sensor_msgs::msg::NavSatFix fix;
static geometry_msgs::msg::Vector3Stamped enu_vel;

static eagleye_msgs::msg::Position enu_absolute_rtk_deadreckoning;
static sensor_msgs::msg::NavSatFix eagleye_fix;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;

rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub1;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub2;

struct RtkDeadreckoningParameter rtk_deadreckoning_parameter;
struct RtkDeadreckoningStatus rtk_deadreckoning_status;

static std::string use_gnss_mode;

rclcpp::Clock clock_(RCL_ROS_TIME);
tf2_ros::Buffer tfBuffer_(std::make_shared<rclcpp::Clock>(clock_));

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  fix = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd = *msg;
}


void on_timer()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform(rtk_deadreckoning_parameter.tf_gnss_parent_flame, rtk_deadreckoning_parameter.tf_gnss_child_flame, tf2::TimePointZero);

    rtk_deadreckoning_parameter.tf_gnss_translation_x = transformStamped.transform.translation.x;
    rtk_deadreckoning_parameter.tf_gnss_translation_y = transformStamped.transform.translation.y;
    rtk_deadreckoning_parameter.tf_gnss_translation_z = transformStamped.transform.translation.z;
    rtk_deadreckoning_parameter.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
    rtk_deadreckoning_parameter.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
    rtk_deadreckoning_parameter.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
    rtk_deadreckoning_parameter.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
  }
  catch (tf2::TransformException& ex)
  {
    // RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }
}

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  rclcpp::Time ros_clock(fix.header.stamp);
  auto fix_time = ros_clock.seconds();

  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_rtk_deadreckoning.header = msg->header;
  enu_absolute_rtk_deadreckoning.header.frame_id = "base_link";
  eagleye_fix.header = msg->header;
  eagleye_fix.header.frame_id = "gnss";
  if (use_gnss_mode == "rtklib" || use_gnss_mode == "RTKLIB") // use RTKLIB mode
    rtk_deadreckoning_estimate(rtklib_nav,enu_vel,fix,heading_interpolate_3rd,rtk_deadreckoning_parameter,&rtk_deadreckoning_status,&enu_absolute_rtk_deadreckoning,&eagleye_fix);
  else if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
    rtk_deadreckoning_estimate(enu_vel,fix,heading_interpolate_3rd,rtk_deadreckoning_parameter,&rtk_deadreckoning_status,&enu_absolute_rtk_deadreckoning,&eagleye_fix);
  if (enu_absolute_rtk_deadreckoning.status.enabled_status == true)
  {
    pub1->publish(enu_absolute_rtk_deadreckoning);
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
  auto node = rclcpp::Node::make_shared("rtk_deadreckoning");

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_navsatfix_topic_name = "/navsat/fix";

  node->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->declare_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->declare_parameter("ecef_base_pos.x", rtk_deadreckoning_parameter.ecef_base_pos_x);
  node->declare_parameter("ecef_base_pos.y", rtk_deadreckoning_parameter.ecef_base_pos_y);
  node->declare_parameter("ecef_base_pos.z", rtk_deadreckoning_parameter.ecef_base_pos_z);
  node->declare_parameter("ecef_base_pos.use_ecef_base_position", rtk_deadreckoning_parameter.use_ecef_base_position);
  node->declare_parameter("rtk_deadreckoning.stop_judgment_velocity_threshold", rtk_deadreckoning_parameter.stop_judgment_velocity_threshold);
  node->declare_parameter("tf_gnss_flame.parent", rtk_deadreckoning_parameter.tf_gnss_parent_flame);
  node->declare_parameter("tf_gnss_flame.child", rtk_deadreckoning_parameter.tf_gnss_child_flame);

  node->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->get_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->get_parameter("ecef_base_pos.x", rtk_deadreckoning_parameter.ecef_base_pos_x);
  node->get_parameter("ecef_base_pos.y", rtk_deadreckoning_parameter.ecef_base_pos_y);
  node->get_parameter("ecef_base_pos.z", rtk_deadreckoning_parameter.ecef_base_pos_z);
  node->get_parameter("ecef_base_pos.use_ecef_base_position", rtk_deadreckoning_parameter.use_ecef_base_position);
  node->get_parameter("rtk_deadreckoning.stop_judgment_velocity_threshold", rtk_deadreckoning_parameter.stop_judgment_velocity_threshold);
  node->get_parameter("tf_gnss_flame.parent", rtk_deadreckoning_parameter.tf_gnss_parent_flame);
  node->get_parameter("tf_gnss_flame.child", rtk_deadreckoning_parameter.tf_gnss_child_flame);
  node->get_parameter("use_gnss_mode",use_gnss_mode);

  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;
  std::cout<< "ecef_base_pos_x "<<rtk_deadreckoning_parameter.ecef_base_pos_x<<std::endl;
  std::cout<< "ecef_base_pos_y "<<rtk_deadreckoning_parameter.ecef_base_pos_y<<std::endl;
  std::cout<< "ecef_base_pos_z "<<rtk_deadreckoning_parameter.ecef_base_pos_z<<std::endl;
  std::cout<< "use_ecef_base_position "<<rtk_deadreckoning_parameter.use_ecef_base_position<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<rtk_deadreckoning_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "tf_gnss_flame.parent "<<rtk_deadreckoning_parameter.tf_gnss_parent_flame<<std::endl;
  std::cout<< "tf_gnss_flame.child "<<rtk_deadreckoning_parameter.tf_gnss_child_flame<<std::endl;
  std::cout<< "use_gnss_mode "<<use_gnss_mode<<std::endl;

  auto sub1 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub2 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000, enu_vel_callback);
  auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_navsatfix_topic_name, 1000, fix_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);
  
  pub1 = node->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_rtk_deadreckoning", 1000);
  pub2 = node->create_publisher<sensor_msgs::msg::NavSatFix>("rtk_fix", 1000);

  const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5));
  auto timer_callback = std::bind(on_timer);
  auto timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    node->get_clock(), period_ns, std::move(timer_callback),
    node->get_node_base_interface()->get_context());
  node->get_node_timers_interface()->add_timer(timer, nullptr);

  rclcpp::spin(node);

  return 0;
}
