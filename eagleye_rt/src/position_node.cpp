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
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Distance distance;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::Position enu_absolute_pos;
static geometry_msgs::msg::Vector3Stamped enu_vel;
static nmea_msgs::msg::Gpgga gga;
rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub;

struct PositionParameter position_parameter;
struct PositionStatus position_status;

std::string use_gnss_mode;
static bool use_can_less_mode;

rclcpp::Clock clock_(RCL_ROS_TIME);
tf2_ros::Buffer tfBuffer_(std::make_shared<rclcpp::Clock>(clock_));

std::string node_name = "eagleye_position";

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd = *msg;
}

void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
{
  gga = *msg;
}


void on_timer()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform(position_parameter.tf_gnss_parent_frame, position_parameter.tf_gnss_child_frame, tf2::TimePointZero);

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
    RCLCPP_WARN(rclcpp::get_logger(node_name), "%s", ex.what());
    return;
  }
}

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  if(use_can_less_mode && !velocity_status.status.enabled_status) return;

  eagleye_msgs::msg::StatusStamped velocity_enable_status;
  if(use_can_less_mode)
  {
    velocity_enable_status = velocity_status;
  }
  else
  {
    velocity_enable_status.header = velocity_scale_factor.header;
    velocity_enable_status.status = velocity_scale_factor.status;
  }

  enu_vel = *msg;
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.header.frame_id = "base_link";
  if (use_gnss_mode == "rtklib" || use_gnss_mode == "RTKLIB") // use RTKLIB mode
    position_estimate(rtklib_nav, velocity, velocity_enable_status, distance, heading_interpolate_3rd, enu_vel,
      position_parameter, &position_status, &enu_absolute_pos);
  else if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
    position_estimate(gga, velocity, velocity_enable_status, distance, heading_interpolate_3rd, enu_vel,
      position_parameter, &position_status, &enu_absolute_pos);
  if (enu_absolute_pos.status.estimate_status == true)
  {
    pub->publish(enu_absolute_pos);
  }
  enu_absolute_pos.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(node_name);

  std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
  std::string subscribe_gga_topic_name = "gnss/gga";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    use_gnss_mode = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();
    use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();

    position_parameter.ecef_base_pos_x = conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
    position_parameter.ecef_base_pos_y = conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
    position_parameter.ecef_base_pos_z = conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();

    position_parameter.tf_gnss_parent_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["parent"].as<std::string>();
    position_parameter.tf_gnss_child_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["child"].as<std::string>();

    position_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    position_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    position_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();

    position_parameter.estimated_interval = conf["/**"]["ros__parameters"]["position"]["estimated_interval"].as<double>();
    position_parameter.update_distance = conf["/**"]["ros__parameters"]["position"]["update_distance"].as<double>();
    position_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["position"]["outlier_threshold"].as<double>();

    position_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["heading"]["gnss_receiving_threshold"].as<double>();
    position_parameter.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["position"]["outlier_ratio_threshold"].as<double>();

    std::cout<< "use_gnss_mode " << use_gnss_mode << std::endl;
    std::cout<< "use_can_less_mode " << use_can_less_mode << std::endl;

    std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;

    std::cout<< "ecef_base_pos_x " << position_parameter.ecef_base_pos_x << std::endl;
    std::cout<< "ecef_base_pos_y " << position_parameter.ecef_base_pos_y << std::endl;
    std::cout<< "ecef_base_pos_z " << position_parameter.ecef_base_pos_z << std::endl;

    std::cout<< "tf_gnss_frame/parent " << position_parameter.tf_gnss_parent_frame << std::endl;
    std::cout<< "tf_gnss_frame/child " << position_parameter.tf_gnss_child_frame << std::endl;

    std::cout << "imu_rate " << position_parameter.imu_rate << std::endl;
    std::cout << "gnss_rate " << position_parameter.gnss_rate << std::endl;
    std::cout << "moving_judgment_threshold " << position_parameter.moving_judgment_threshold << std::endl;

    std::cout << "estimated_interval " << position_parameter.estimated_interval << std::endl;
    std::cout << "update_distance " << position_parameter.update_distance << std::endl;
    std::cout << "outlier_threshold " << position_parameter.outlier_threshold << std::endl;
    std::cout << "gnss_receiving_threshold " << position_parameter.gnss_receiving_threshold << std::endl;
    std::cout << "outlier_ratio_threshold " << position_parameter.outlier_ratio_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mposition Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }


  auto sub1 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000, enu_vel_callback);
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub3 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", 1000, velocity_scale_factor_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", 1000, distance_callback);
  auto sub7 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);
  auto sub8 = node->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, gga_callback);

  pub = node->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos", 1000);

  const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5));
  auto timer_callback = std::bind(on_timer);
  auto timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    node->get_clock(), period_ns, std::move(timer_callback),
    node->get_node_base_interface()->get_context());
  node->get_node_timers_interface()->add_timer(timer, nullptr);

  tf2_ros::TransformListener listener(tfBuffer_);

  rclcpp::spin(node);

  return 0;
}
