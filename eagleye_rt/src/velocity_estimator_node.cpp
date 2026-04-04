// Copyright (c) 2022, Map IV, Inc.
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
 * velocity_estimator_node.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

#include <yaml-cpp/yaml.h>

class VelocityEstimatorNode : public rclcpp::Node
{
public:
  VelocityEstimatorNode() : Node("eagleye_velocity_estimator")
  {
    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);

    velocity_estimator_.setParam(yaml_file);

    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      "gnss/rtklib_nav", 1000,
      std::bind(&VelocityEstimatorNode::rtklibNavCallback, this, std::placeholders::_1));
    sub_gga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      "gnss/gga", 1000,
      std::bind(&VelocityEstimatorNode::ggaCallback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&VelocityEstimatorNode::imuCallback, this, std::placeholders::_1));

    pub_velocity_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 1000);
    pub_velocity_status_ =
      this->create_publisher<eagleye_msgs::msg::StatusStamped>("velocity_status", 1000);
  }

private:
  rtklib_msgs::msg::RtklibNav rtklib_nav_msg_;
  nmea_msgs::msg::Gpgga gga_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  geometry_msgs::msg::TwistStamped velocity_msg_;
  VelocityEstimator velocity_estimator_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<eagleye_msgs::msg::StatusStamped>::SharedPtr pub_velocity_status_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_msg_ = *msg;
  }

  void ggaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg) { gga_msg_ = *msg; }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    imu_msg_ = *msg;

    velocity_estimator_.VelocityEstimate(imu_msg_, rtklib_nav_msg_, gga_msg_, &velocity_msg_);

    eagleye_msgs::msg::StatusStamped velocity_status;
    velocity_status.header = msg->header;
    velocity_status.status = velocity_estimator_.getStatus();
    pub_velocity_status_->publish(velocity_status);

    if (velocity_status.status.enabled_status) {
      pub_velocity_->publish(velocity_msg_);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityEstimatorNode>());
  return 0;
}
