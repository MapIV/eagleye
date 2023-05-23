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
 * rtk_dead_reckoning_node.cpp
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
static nmea_msgs::msg::Gpgga gga;
static geometry_msgs::msg::Vector3Stamped enu_vel;

static eagleye_msgs::msg::Position enu_absolute_rtk_dead_reckoning;
static sensor_msgs::msg::NavSatFix eagleye_fix;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;

rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub1;
rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub2;

struct RtkDeadreckoningParameter rtk_dead_reckoning_parameter;
struct RtkDeadreckoningStatus rtk_dead_reckoning_status;

std::string use_gnss_mode;

rclcpp::Clock clock_(RCL_ROS_TIME);
tf2_ros::Buffer tfBuffer_(std::make_shared<rclcpp::Clock>(clock_));

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
{
  gga = *msg;
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
    transformStamped = tfBuffer_.lookupTransform(rtk_dead_reckoning_parameter.tf_gnss_parent_frame, rtk_dead_reckoning_parameter.tf_gnss_child_frame, tf2::TimePointZero);

    rtk_dead_reckoning_parameter.tf_gnss_translation_x = transformStamped.transform.translation.x;
    rtk_dead_reckoning_parameter.tf_gnss_translation_y = transformStamped.transform.translation.y;
    rtk_dead_reckoning_parameter.tf_gnss_translation_z = transformStamped.transform.translation.z;
    rtk_dead_reckoning_parameter.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
    rtk_dead_reckoning_parameter.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
    rtk_dead_reckoning_parameter.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
    rtk_dead_reckoning_parameter.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
  }
  catch (tf2::TransformException& ex)
  {
    // RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }
}

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  rclcpp::Time ros_clock(gga.header.stamp);
  auto gga_time = ros_clock.seconds();

  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_rtk_dead_reckoning.header = msg->header;
  enu_absolute_rtk_dead_reckoning.header.frame_id = "base_link";
  eagleye_fix.header = msg->header;
  eagleye_fix.header.frame_id = "gnss";
  if (use_gnss_mode == "rtklib" || use_gnss_mode == "RTKLIB") // use RTKLIB mode
    rtk_dead_reckoning_estimate(rtklib_nav,enu_vel,gga,heading_interpolate_3rd,rtk_dead_reckoning_parameter,&rtk_dead_reckoning_status,&enu_absolute_rtk_dead_reckoning,&eagleye_fix);
  else if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
    rtk_dead_reckoning_estimate(enu_vel,gga,heading_interpolate_3rd,rtk_dead_reckoning_parameter,&rtk_dead_reckoning_status,&enu_absolute_rtk_dead_reckoning,&eagleye_fix);
  if (enu_absolute_rtk_dead_reckoning.status.enabled_status == true)
  {
    pub1->publish(enu_absolute_rtk_dead_reckoning);
    pub2->publish(eagleye_fix);
  }
  else if (gga_time != 0)
  {
    sensor_msgs::msg::NavSatFix fix;
    fix.header = gga.header;
    fix.latitude = gga.lat;
    fix.longitude = gga.lon;
    fix.altitude = gga.alt + gga.undulation;
    pub2->publish(fix);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_rtk_dead_reckoning");

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

    rtk_dead_reckoning_parameter.ecef_base_pos_x = conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
    rtk_dead_reckoning_parameter.ecef_base_pos_y = conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
    rtk_dead_reckoning_parameter.ecef_base_pos_z = conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();
    rtk_dead_reckoning_parameter.use_ecef_base_position = conf["/**"]["ros__parameters"]["ecef_base_pos"]["use_ecef_base_position"].as<bool>();
    rtk_dead_reckoning_parameter.tf_gnss_parent_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["parent"].as<std::string>();
    rtk_dead_reckoning_parameter.tf_gnss_child_frame = conf["/**"]["ros__parameters"]["tf_gnss_frame"]["child"].as<std::string>();
    rtk_dead_reckoning_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    rtk_dead_reckoning_parameter.rtk_fix_STD = conf["/**"]["ros__parameters"]["rtk_dead_reckoning"]["rtk_fix_STD"].as<double>();
    rtk_dead_reckoning_parameter.proc_noise = conf["/**"]["ros__parameters"]["rtk_dead_reckoning"]["proc_noise"].as<double>();

    std::cout << "use_gnss_mode " << use_gnss_mode << std::endl;

    std::cout << "ecef_base_pos_x " << rtk_dead_reckoning_parameter.ecef_base_pos_x << std::endl;
    std::cout << "ecef_base_pos_y " << rtk_dead_reckoning_parameter.ecef_base_pos_y << std::endl;
    std::cout << "ecef_base_pos_z " << rtk_dead_reckoning_parameter.ecef_base_pos_z << std::endl;
    std::cout << "use_ecef_base_position " << rtk_dead_reckoning_parameter.use_ecef_base_position << std::endl;
    std::cout << "tf_gnss_frame/parent " << rtk_dead_reckoning_parameter.tf_gnss_parent_frame << std::endl;
    std::cout << "tf_gnss_frame/child " << rtk_dead_reckoning_parameter.tf_gnss_child_frame << std::endl;
    std::cout << "stop_judgment_threshold " << rtk_dead_reckoning_parameter.stop_judgment_threshold << std::endl;
    std::cout << "rtk_fix_STD " << rtk_dead_reckoning_parameter.rtk_fix_STD << std::endl;
    std::cout << "proc_noise " << rtk_dead_reckoning_parameter.proc_noise << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mrtk_dead_reckoning Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  auto sub1 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub2 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000, enu_vel_callback);
  auto sub3 = node->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, gga_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);
  
  pub1 = node->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos_interpolate", 1000);
  pub2 = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1000);

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
