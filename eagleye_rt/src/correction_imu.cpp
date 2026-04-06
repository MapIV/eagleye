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
 * correction_imu.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class CorrectionImuNode : public rclcpp::Node
{
public:
  CorrectionImuNode() : Node("eagleye_correction_imu")
  {
    sub1_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_2nd", rclcpp::QoS(10),
      std::bind(&CorrectionImuNode::yawRateOffsetCallback, this, std::placeholders::_1));
    sub2_ = this->create_subscription<eagleye_msgs::msg::AngularVelocityOffset>(
      "angular_velocity_offset_stop", rclcpp::QoS(10),
      std::bind(&CorrectionImuNode::angularVelocityOffsetStopCallback, this, std::placeholders::_1));
    sub3_ = this->create_subscription<eagleye_msgs::msg::AccXOffset>(
      "acc_x_offset", rclcpp::QoS(10),
      std::bind(&CorrectionImuNode::accXOffsetCallback, this, std::placeholders::_1));
    sub4_ = this->create_subscription<eagleye_msgs::msg::AccXScaleFactor>(
      "acc_x_scale_factor", rclcpp::QoS(10),
      std::bind(&CorrectionImuNode::accXScaleFactorCallback, this, std::placeholders::_1));
    sub5_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&CorrectionImuNode::imuCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_corrected", rclcpp::QoS(10));
  }

private:
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_;
  eagleye_msgs::msg::AngularVelocityOffset angular_velocity_offset_stop_;
  eagleye_msgs::msg::AccXOffset acc_x_offset_;
  eagleye_msgs::msg::AccXScaleFactor acc_x_scale_factor_;
  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::Imu correction_imu_;

  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub1_;
  rclcpp::Subscription<eagleye_msgs::msg::AngularVelocityOffset>::SharedPtr sub2_;
  rclcpp::Subscription<eagleye_msgs::msg::AccXOffset>::SharedPtr sub3_;
  rclcpp::Subscription<eagleye_msgs::msg::AccXScaleFactor>::SharedPtr sub4_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub5_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  void yawRateOffsetCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_ = *msg;
  }

  void angularVelocityOffsetStopCallback(const eagleye_msgs::msg::AngularVelocityOffset::ConstSharedPtr msg)
  {
    angular_velocity_offset_stop_ = *msg;
  }

  void accXOffsetCallback(const eagleye_msgs::msg::AccXOffset::ConstSharedPtr msg)
  {
    acc_x_offset_ = *msg;
  }

  void accXScaleFactorCallback(const eagleye_msgs::msg::AccXScaleFactor::ConstSharedPtr msg)
  {
    acc_x_scale_factor_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    imu_ = *msg;

    correction_imu_.header = imu_.header;
    correction_imu_.orientation = imu_.orientation;
    correction_imu_.orientation_covariance = imu_.orientation_covariance;
    correction_imu_.angular_velocity_covariance = imu_.angular_velocity_covariance;
    correction_imu_.linear_acceleration_covariance = imu_.linear_acceleration_covariance;

    if (acc_x_offset_.status.enabled_status == true && acc_x_scale_factor_.status.enabled_status)
    {
      correction_imu_.linear_acceleration.x = imu_.linear_acceleration.x * acc_x_scale_factor_.acc_x_scale_factor + acc_x_offset_.acc_x_offset;
      correction_imu_.linear_acceleration.y = imu_.linear_acceleration.y;
      correction_imu_.linear_acceleration.z = imu_.linear_acceleration.z;
    }
    else
    {
      correction_imu_.linear_acceleration.x = imu_.linear_acceleration.x;
      correction_imu_.linear_acceleration.y = imu_.linear_acceleration.y;
      correction_imu_.linear_acceleration.z = imu_.linear_acceleration.z;
    }

    correction_imu_.angular_velocity.x = imu_.angular_velocity.x + angular_velocity_offset_stop_.angular_velocity_offset.x;
    correction_imu_.angular_velocity.y = imu_.angular_velocity.y + angular_velocity_offset_stop_.angular_velocity_offset.y;
    correction_imu_.angular_velocity.z = -1 * (imu_.angular_velocity.z + angular_velocity_offset_stop_.angular_velocity_offset.z);

    pub_->publish(correction_imu_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CorrectionImuNode>());
  return 0;
}
