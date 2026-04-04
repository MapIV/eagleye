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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

class SlipCoefficientNode : public rclcpp::Node
{
public:
  SlipCoefficientNode() : Node("eagleye_slip_coefficient")
  {
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      slip_coefficient_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      slip_coefficient_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      slip_coefficient_parameter_.moving_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
      slip_coefficient_parameter_.estimated_minimum_interval =
        conf["/**"]["ros__parameters"]["slip_coefficient"]["estimated_minimum_interval"]
          .as<double>();
      slip_coefficient_parameter_.estimated_maximum_interval =
        conf["/**"]["ros__parameters"]["slip_coefficient"]["estimated_maximum_interval"]
          .as<double>();
      slip_coefficient_parameter_.curve_judgment_threshold =
        conf["/**"]["ros__parameters"]["slip_coefficient"]["curve_judgment_threshold"].as<double>();
      slip_coefficient_parameter_.lever_arm =
        conf["/**"]["ros__parameters"]["slip_coefficient"]["lever_arm"].as<double>();

      std::cout << "use_can_less_mode " << use_can_less_mode_ << std::endl;
      std::cout << "imu_rate " << slip_coefficient_parameter_.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold "
                << slip_coefficient_parameter_.stop_judgment_threshold << std::endl;
      std::cout << "moving_judgment_threshold "
                << slip_coefficient_parameter_.moving_judgment_threshold << std::endl;
      std::cout << "estimated_minimum_interval "
                << slip_coefficient_parameter_.estimated_minimum_interval << std::endl;
      std::cout << "estimated_maximum_interval "
                << slip_coefficient_parameter_.estimated_maximum_interval << std::endl;
      std::cout << "curve_judgment_threshold "
                << slip_coefficient_parameter_.curve_judgment_threshold << std::endl;
      std::cout << "lever_arm " << slip_coefficient_parameter_.lever_arm << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mslip_coefficient Node YAML Error: " << e.msg << "\033[0m"
                << std::endl;
      exit(3);
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&SlipCoefficientNode::imuCallback, this, std::placeholders::_1));
    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      subscribe_rtklib_nav_topic_name, 1000,
      std::bind(&SlipCoefficientNode::rtklibNavCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&SlipCoefficientNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&SlipCoefficientNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(&SlipCoefficientNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_2nd_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_2nd", rclcpp::QoS(10),
      std::bind(&SlipCoefficientNode::yawRateOffset2ndCallback, this, std::placeholders::_1));
    sub_heading_interpolate_3rd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_3rd", rclcpp::QoS(10),
      std::bind(
        &SlipCoefficientNode::headingInterpolate3rdCallback, this, std::placeholders::_1));
  }

  double getEstimateCoefficient() const { return estimate_coefficient_; }

private:
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  SlipCoefficientParameter slip_coefficient_parameter_;
  SlipCoefficientStatus slip_coefficient_status_;
  double estimate_coefficient_ = 0.0;
  bool is_first_correction_velocity_ = false;
  bool use_can_less_mode_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_3rd_;

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_ = *msg;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
    if (
      is_first_correction_velocity_ == false &&
      msg->twist.linear.x > slip_coefficient_parameter_.moving_judgment_threshold) {
      is_first_correction_velocity_ = true;
    }
  }

  void velocityStatusCallback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status_ = *msg;
  }

  void yawRateOffsetStopCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop_ = *msg;
  }

  void yawRateOffset2ndCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd_ = *msg;
  }

  void headingInterpolate3rdCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_3rd_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (is_first_correction_velocity_ == false) return;
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    imu_ = *msg;
    slip_coefficient_estimate(
      imu_, rtklib_nav_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_2nd_,
      heading_interpolate_3rd_, slip_coefficient_parameter_, &slip_coefficient_status_,
      &estimate_coefficient_);

    std::cout << "--- \033[1;34m slip_coefficient \033[m ------------------------------"
              << std::endl;
    std::cout << "\033[1m estimate_coefficient \033[m " << estimate_coefficient_ << std::endl;
    std::cout << std::endl;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlipCoefficientNode>();
  rclcpp::spin(node);

  std::string str;
  node->declare_parameter(str, "output_dir");
  std::ofstream ofs(str, std::ios_base::trunc | std::ios_base::out);
  ofs << "slip_coefficient"
      << " : " << node->getEstimateCoefficient() << std::endl;
  ofs.close();

  return 0;
}
