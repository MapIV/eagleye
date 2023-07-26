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
 * heading.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static nmea_msgs::msg::Gpgga gga;
static sensor_msgs::msg::Imu imu;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
static eagleye_msgs::msg::Distance distance;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset;
static eagleye_msgs::msg::SlipAngle slip_angle;
static eagleye_msgs::msg::Heading heading_interpolate;

rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub;
static eagleye_msgs::msg::Heading heading;

struct RtkHeadingParameter heading_parameter;
struct RtkHeadingStatus heading_status;

static bool use_can_less_mode;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
{
  gga = *msg;
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

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate = *msg;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(use_can_less_mode && !velocity_status.status.enabled_status) return;

  imu = *msg;
  heading.header = msg->header;
  heading.header.frame_id = "base_link";
  rtk_heading_estimate(gga,imu,velocity,distance,yaw_rate_offset_stop,yaw_rate_offset,slip_angle,heading_interpolate,heading_parameter,&heading_status,&heading);

  if (heading.status.estimate_status == true)
  {
    pub->publish(heading);
  }
  heading.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_rtk_heading");


  std::string subscribe_gga_topic_name = "gnss/gga";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;
  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();

    heading_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    heading_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    heading_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    heading_parameter.slow_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["slow_judgment_threshold"].as<double>();

    heading_parameter.update_distance = conf["/**"]["ros__parameters"]["rtk_heading"]["update_distance"].as<double>();
    heading_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["rtk_heading"]["estimated_minimum_interval"].as<double>();
    heading_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["rtk_heading"]["estimated_maximum_interval"].as<double>();
    heading_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["rtk_heading"]["gnss_receiving_threshold"].as<double>();
    heading_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["rtk_heading"]["outlier_threshold"].as<double>();
    heading_parameter.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["rtk_heading"]["outlier_ratio_threshold"].as<double>();
    heading_parameter.curve_judgment_threshold = conf["/**"]["ros__parameters"]["rtk_heading"]["curve_judgment_threshold"].as<double>();

    std::cout<< "use_can_less_mode " << use_can_less_mode << std::endl;

    std::cout<< "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;

    std::cout << "imu_rate " << heading_parameter.imu_rate << std::endl;
    std::cout << "gnss_rate " << heading_parameter.gnss_rate << std::endl;
    std::cout << "stop_judgment_threshold " << heading_parameter.stop_judgment_threshold << std::endl;
    std::cout << "slow_judgment_threshold " << heading_parameter.slow_judgment_threshold << std::endl;

    std::cout << "update_distance " << heading_parameter.update_distance << std::endl;
    std::cout << "estimated_minimum_interval " << heading_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << heading_parameter.estimated_maximum_interval << std::endl;
    std::cout << "gnss_receiving_threshold " << heading_parameter.gnss_receiving_threshold << std::endl;
    std::cout << "outlier_threshold " << heading_parameter.outlier_threshold << std::endl;
    std::cout << "outlier_ratio_threshold " << heading_parameter.outlier_ratio_threshold << std::endl;
    std::cout << "curve_judgment_threshold " << heading_parameter.curve_judgment_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mrtk_heading Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";
  std::string subscribe_topic_name2 = "/subscribe_topic_name2/invalid";

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "heading_1st";
      subscribe_topic_name = "yaw_rate_offset_stop";
      subscribe_topic_name2 = "heading_interpolate_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "heading_2nd";
      subscribe_topic_name = "yaw_rate_offset_1st";
      subscribe_topic_name2 = "heading_interpolate_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "heading_3rd";
      subscribe_topic_name = "yaw_rate_offset_2nd";
      subscribe_topic_name2 = "heading_interpolate_3rd";
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
  auto sub2 = node->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, gga_callback);
  auto sub3 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", 1000, yaw_rate_offset_stop_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>(subscribe_topic_name, 1000, yaw_rate_offset_callback);
  auto sub7 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", 1000, slip_angle_callback);
  auto sub8 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name2, 1000, heading_interpolate_callback);
  auto sub9 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", 1000, distance_callback);

  pub = node->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, 1000);

  rclcpp::spin(node);

  return 0;
}
