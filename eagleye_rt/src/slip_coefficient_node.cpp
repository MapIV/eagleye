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
 * slip_coefficient.cpp
 * Author MapIV Takanose
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static sensor_msgs::msg::Imu imu;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;

struct SlipCoefficientParameter slip_coefficient_parameter;
struct SlipCoefficientStatus slip_coefficient_status;

static double estimate_coefficient;

bool is_first_correction_velocity = false;
static bool use_can_less_mode;

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
  if (is_first_correction_velocity == false && msg->twist.linear.x > slip_coefficient_parameter.moving_judgment_threshold)
  {
    is_first_correction_velocity = true;
  }
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset_stop = *msg;
}

void yaw_rate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset_2nd = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (is_first_correction_velocity == false) return;
  if (use_can_less_mode && !velocity_status.status.enabled_status) return;

  imu = *msg;
  slip_coefficient_estimate(imu,rtklib_nav,velocity,yaw_rate_offset_stop,yaw_rate_offset_2nd,heading_interpolate_3rd,slip_coefficient_parameter,&slip_coefficient_status,&estimate_coefficient);

  std::cout << "--- \033[1;34m slip_coefficient \033[m ------------------------------"<< std::endl;
  std::cout<<"\033[1m estimate_coefficient \033[m "<<estimate_coefficient<<std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_slip_coefficient");

  std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    use_can_less_mode = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();

    slip_coefficient_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    slip_coefficient_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    slip_coefficient_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();

    slip_coefficient_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["slip_coefficient"]["estimated_minimum_interval"].as<double>();
    slip_coefficient_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["slip_coefficient"]["estimated_maximum_interval"].as<double>();
    slip_coefficient_parameter.curve_judgment_threshold = conf["/**"]["ros__parameters"]["slip_coefficient"]["curve_judgment_threshold"].as<double>();
    slip_coefficient_parameter.lever_arm = conf["/**"]["ros__parameters"]["slip_coefficient"]["lever_arm"].as<double>();

    std::cout<< "use_can_less_mode " << use_can_less_mode << std::endl;

    std::cout << "imu_rate " << slip_coefficient_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << slip_coefficient_parameter.stop_judgment_threshold << std::endl;
    std::cout << "moving_judgment_threshold " << slip_coefficient_parameter.moving_judgment_threshold << std::endl;

    std::cout << "estimated_minimum_interval " << slip_coefficient_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << slip_coefficient_parameter.estimated_maximum_interval << std::endl;
    std::cout << "curve_judgment_threshold " << slip_coefficient_parameter.curve_judgment_threshold << std::endl;
    std::cout << "lever_arm " << slip_coefficient_parameter.lever_arm << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mslip_coefficient Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub3 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), yaw_rate_offset_stop_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_2nd", rclcpp::QoS(10), yaw_rate_offset_2nd_callback);
  auto sub7 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", rclcpp::QoS(10), heading_interpolate_3rd_callback);

  rclcpp::spin(node);

  std::string str;
  node->declare_parameter(str, "output_dir");
  std::ofstream ofs(str, std::ios_base::trunc | std::ios_base::out);
  ofs << "slip_coefficient" << " : " << estimate_coefficient << std::endl;
  ofs.close();

  return 0;
}
