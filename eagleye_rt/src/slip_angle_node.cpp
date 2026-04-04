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
 * slip_angle.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class SlipAngleNode : public rclcpp::Node
{
public:
  SlipAngleNode() : Node("eagleye_slip_angle")
  {
    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      slip_angle_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      slip_angle_parameter_.manual_coefficient =
        conf["/**"]["ros__parameters"]["slip_angle"]["manual_coefficient"].as<double>();

      std::cout << "stop_judgment_threshold " << slip_angle_parameter_.stop_judgment_threshold
                << std::endl;
      std::cout << "manual_coefficient " << slip_angle_parameter_.manual_coefficient << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mslip_angle Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", rclcpp::QoS(10),
      std::bind(&SlipAngleNode::imuCallback, this, std::placeholders::_1));
    sub_velocity_scale_factor_ = this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
      "velocity_scale_factor", rclcpp::QoS(10),
      std::bind(&SlipAngleNode::velocityScaleFactorCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(&SlipAngleNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_2nd_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_2nd", rclcpp::QoS(10),
      std::bind(&SlipAngleNode::yawRateOffset2ndCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&SlipAngleNode::velocityCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10));
  }

private:
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  SlipangleParameter slip_angle_parameter_;
  bool use_can_less_mode_ = false;

  rclcpp::Publisher<eagleye_msgs::msg::SlipAngle>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr
    sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_2nd_;

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void velocityStatusCallback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status_ = *msg;
  }

  void velocityScaleFactorCallback(
    const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
  {
    velocity_scale_factor_ = *msg;
  }

  void yawRateOffsetStopCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop_ = *msg;
  }

  void yawRateOffset2ndCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    eagleye_msgs::msg::StatusStamped velocity_enable_status;
    if (use_can_less_mode_) {
      velocity_enable_status = velocity_status_;
    } else {
      velocity_enable_status.header = velocity_scale_factor_.header;
      velocity_enable_status.status = velocity_scale_factor_.status;
    }

    imu_ = *msg;
    slip_angle_.header = msg->header;
    slip_angle_.header.frame_id = "base_link";
    slip_angle_estimate(
      imu_, velocity_, velocity_enable_status, yaw_rate_offset_stop_, yaw_rate_offset_2nd_,
      slip_angle_parameter_, &slip_angle_);
    pub_->publish(slip_angle_);
    slip_angle_.status.estimate_status = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlipAngleNode>());
  return 0;
}
