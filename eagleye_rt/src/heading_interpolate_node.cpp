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
 * heading_interpolate.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static sensor_msgs::msg::Imu imu;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset;
static eagleye_msgs::msg::Heading heading;
static eagleye_msgs::msg::SlipAngle slip_angle;

rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub;
static eagleye_msgs::msg::Heading heading_interpolate;

struct HeadingInterpolateParameter heading_interpolate_parameter;
struct HeadingInterpolateStatus heading_interpolate_status;

static bool _use_can_less_mode;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset_stop = *msg;
}

void yaw_rate_offset_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset = *msg;
}

void heading_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading = *msg;
}

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(_use_can_less_mode && !velocity_status.status.enabled_status) return;

  imu = *msg;
  heading_interpolate.header = msg->header;
  heading_interpolate.header.frame_id = "base_link";
  heading_interpolate_estimate(imu,velocity,yaw_rate_offset_stop,yaw_rate_offset,heading,slip_angle,heading_interpolate_parameter,
    &heading_interpolate_status,&heading_interpolate);
  pub->publish(heading_interpolate);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_heading_interpolate");

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    heading_interpolate_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    heading_interpolate_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    heading_interpolate_parameter.sync_search_period = conf["/**"]["ros__parameters"]["heading_interpolate"]["sync_search_period"].as<double>();
    heading_interpolate_parameter.proc_noise = conf["/**"]["ros__parameters"]["heading_interpolate"]["proc_noise"].as<double>();

    std::cout << "imu_rate " << heading_interpolate_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << heading_interpolate_parameter.stop_judgment_threshold << std::endl;
    std::cout << "sync_search_period " << heading_interpolate_parameter.sync_search_period << std::endl;
    std::cout << "proc_noise " << heading_interpolate_parameter.proc_noise << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mheading_interpolate Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name_1 = "/subscribe_topic_name/invalid_1";
  std::string subscribe_topic_name_2 = "/subscribe_topic_name/invalid_2";

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "heading_interpolate_1st";
      subscribe_topic_name_1 = "yaw_rate_offset_stop";
      subscribe_topic_name_2 = "heading_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "heading_interpolate_2nd";
      subscribe_topic_name_1 = "yaw_rate_offset_1st";
      subscribe_topic_name_2 = "heading_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "heading_interpolate_3rd";
      subscribe_topic_name_1 = "yaw_rate_offset_2nd";
      subscribe_topic_name_2 = "heading_3rd";
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(),"Invalid argument");
      rclcpp::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(),"No arguments");
    rclcpp::shutdown();
  }

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto sub2 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub3 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), yaw_rate_offset_stop_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>(subscribe_topic_name_1, 1000, yaw_rate_offset_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name_2, 1000, heading_callback);
  auto sub7 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10), slip_angle_callback);
  pub = node->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
