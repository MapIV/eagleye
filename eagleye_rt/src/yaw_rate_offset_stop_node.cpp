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
 * yaw_rate_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class YawRateOffsetStopNode : public rclcpp::Node
{
public:
  YawRateOffsetStopNode() : Node("eagleye_yaw_rate_offset_stop")
  {
    std::string subscribe_twist_topic_name = "vehicle/twist";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      yaw_rate_offset_stop_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      yaw_rate_offset_stop_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      yaw_rate_offset_stop_parameter_.estimated_interval =
        conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["estimated_interval"].as<double>();
      yaw_rate_offset_stop_parameter_.outlier_threshold =
        conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

      std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;
      std::cout << "imu_rate " << yaw_rate_offset_stop_parameter_.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold "
                << yaw_rate_offset_stop_parameter_.stop_judgment_threshold << std::endl;
      std::cout << "estimated_minimum_interval "
                << yaw_rate_offset_stop_parameter_.estimated_interval << std::endl;
      std::cout << "outlier_threshold " << yaw_rate_offset_stop_parameter_.outlier_threshold
                << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31myaw_rate_offset_stop Node YAML Error: " << e.msg << "\033[0m"
                << std::endl;
      exit(3);
    }

    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      subscribe_twist_topic_name, 1000,
      std::bind(&YawRateOffsetStopNode::velocityCallback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&YawRateOffsetStopNode::imuCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10));
  }

private:
  geometry_msgs::msg::TwistStamped::ConstSharedPtr velocity_ptr_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  sensor_msgs::msg::Imu imu_;
  YawrateOffsetStopParameter yaw_rate_offset_stop_parameter_;
  YawrateOffsetStopStatus yaw_rate_offset_stop_status_ = {};
  double previous_yaw_rate_offset_stop_ = 0.0;

  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ptr_ = msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (velocity_ptr_ == nullptr) return;
    imu_ = *msg;
    yaw_rate_offset_stop_.header = msg->header;
    yaw_rate_offset_stop_estimate(
      *velocity_ptr_, imu_, yaw_rate_offset_stop_parameter_, &yaw_rate_offset_stop_status_,
      &yaw_rate_offset_stop_);

    yaw_rate_offset_stop_.status.is_abnormal = false;
    if (!std::isfinite(yaw_rate_offset_stop_.yaw_rate_offset)) {
      yaw_rate_offset_stop_.yaw_rate_offset = previous_yaw_rate_offset_stop_;
      yaw_rate_offset_stop_.status.is_abnormal = true;
      yaw_rate_offset_stop_.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
    } else {
      previous_yaw_rate_offset_stop_ = yaw_rate_offset_stop_.yaw_rate_offset;
    }

    pub_->publish(yaw_rate_offset_stop_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawRateOffsetStopNode>());
  return 0;
}
