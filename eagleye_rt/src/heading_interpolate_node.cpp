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

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class HeadingInterpolateNode : public rclcpp::Node
{
public:
  HeadingInterpolateNode(int argc, char** argv) : Node("eagleye_heading_interpolate")
  {
    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      heading_interpolate_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      heading_interpolate_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      heading_interpolate_parameter_.sync_search_period =
        conf["/**"]["ros__parameters"]["heading_interpolate"]["sync_search_period"].as<double>();
      heading_interpolate_parameter_.proc_noise =
        conf["/**"]["ros__parameters"]["heading_interpolate"]["proc_noise"].as<double>();

      std::cout << "imu_rate " << heading_interpolate_parameter_.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold "
                << heading_interpolate_parameter_.stop_judgment_threshold << std::endl;
      std::cout << "sync_search_period " << heading_interpolate_parameter_.sync_search_period
                << std::endl;
      std::cout << "proc_noise " << heading_interpolate_parameter_.proc_noise << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mheading_interpolate Node YAML Error: " << e.msg << "\033[0m"
                << std::endl;
      exit(3);
    }

    std::string publish_topic_name = "/publish_topic_name/invalid";
    std::string subscribe_topic_name_1 = "/subscribe_topic_name/invalid_1";
    std::string subscribe_topic_name_2 = "/subscribe_topic_name/invalid_2";

    if (argc > 2) {
      if (strcmp(argv[1], "1st") == 0) {
        publish_topic_name = "heading_interpolate_1st";
        subscribe_topic_name_1 = "yaw_rate_offset_stop";
        subscribe_topic_name_2 = "heading_1st";
      } else if (strcmp(argv[1], "2nd") == 0) {
        publish_topic_name = "heading_interpolate_2nd";
        subscribe_topic_name_1 = "yaw_rate_offset_1st";
        subscribe_topic_name_2 = "heading_2nd";
      } else if (strcmp(argv[1], "3rd") == 0) {
        publish_topic_name = "heading_interpolate_3rd";
        subscribe_topic_name_1 = "yaw_rate_offset_2nd";
        subscribe_topic_name_2 = "heading_3rd";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid argument");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "No arguments");
      rclcpp::shutdown();
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&HeadingInterpolateNode::imuCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&HeadingInterpolateNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&HeadingInterpolateNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(
        &HeadingInterpolateNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      subscribe_topic_name_1, 1000,
      std::bind(&HeadingInterpolateNode::yawRateOffsetCallback, this, std::placeholders::_1));
    sub_heading_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      subscribe_topic_name_2, 1000,
      std::bind(&HeadingInterpolateNode::headingCallback, this, std::placeholders::_1));
    sub_slip_angle_ = this->create_subscription<eagleye_msgs::msg::SlipAngle>(
      "slip_angle", rclcpp::QoS(10),
      std::bind(&HeadingInterpolateNode::slipAngleCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, rclcpp::QoS(10));
  }

private:
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_;
  eagleye_msgs::msg::Heading heading_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  eagleye_msgs::msg::Heading heading_interpolate_;
  HeadingInterpolateParameter heading_interpolate_parameter_;
  HeadingInterpolateStatus heading_interpolate_status_ = {};
  bool use_can_less_mode_ = false;

  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_;
  rclcpp::Subscription<eagleye_msgs::msg::SlipAngle>::SharedPtr sub_slip_angle_;

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void velocityStatusCallback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status_ = *msg;
  }

  void yawRateOffsetStopCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop_ = *msg;
  }

  void yawRateOffsetCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_ = *msg;
  }

  void headingCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_ = *msg;
  }

  void slipAngleCallback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
  {
    slip_angle_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    imu_ = *msg;
    heading_interpolate_.header = msg->header;
    heading_interpolate_.header.frame_id = "base_link";
    heading_interpolate_estimate(
      imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_, heading_, slip_angle_,
      heading_interpolate_parameter_, &heading_interpolate_status_, &heading_interpolate_);
    pub_->publish(heading_interpolate_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingInterpolateNode>(argc, argv));
  return 0;
}
