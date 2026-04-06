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
 * yaw_rate_offset.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class YawRateOffsetNode : public rclcpp::Node
{
public:
  YawRateOffsetNode(int argc, char** argv) : Node("eagleye_yaw_rate_offset")
  {
    std::string publish_topic_name = "/publish_topic_name/invalid";
    std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    if (argc > 2) {
      if (strcmp(argv[1], "1st") == 0) {
        publish_topic_name = "yaw_rate_offset_1st";
        subscribe_topic_name = "heading_interpolate_1st";

        try {
          YAML::Node conf = YAML::LoadFile(yaml_file);

          yaw_rate_offset_parameter_.imu_rate =
            conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
          yaw_rate_offset_parameter_.gnss_rate =
            conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
          yaw_rate_offset_parameter_.moving_judgment_threshold =
            conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
          yaw_rate_offset_parameter_.estimated_minimum_interval =
            conf["/**"]["ros__parameters"]["yaw_rate_offset"]["estimated_minimum_interval"]
              .as<double>();
          yaw_rate_offset_parameter_.estimated_maximum_interval =
            conf["/**"]["ros__parameters"]["yaw_rate_offset"]["1st"]["estimated_maximum_interval"]
              .as<double>();
          yaw_rate_offset_parameter_.gnss_receiving_threshold =
            conf["/**"]["ros__parameters"]["yaw_rate_offset"]["gnss_receiving_threshold"]
              .as<double>();
          yaw_rate_offset_parameter_.outlier_threshold =
            conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

          std::cout << "imu_rate " << yaw_rate_offset_parameter_.imu_rate << std::endl;
          std::cout << "gnss_rate " << yaw_rate_offset_parameter_.gnss_rate << std::endl;
          std::cout << "moving_judgment_threshold "
                    << yaw_rate_offset_parameter_.moving_judgment_threshold << std::endl;
          std::cout << "estimated_minimum_interval "
                    << yaw_rate_offset_parameter_.estimated_minimum_interval << std::endl;
          std::cout << "estimated_maximum_interval "
                    << yaw_rate_offset_parameter_.estimated_maximum_interval << std::endl;
          std::cout << "gnss_receiving_threshold "
                    << yaw_rate_offset_parameter_.gnss_receiving_threshold << std::endl;
          std::cout << "outlier_threshold " << yaw_rate_offset_parameter_.outlier_threshold
                    << std::endl;
        } catch (YAML::Exception& e) {
          std::cerr << "\033[1;yaw_rate_offset_1st Node YAML Error: " << e.msg << "\033[0m"
                    << std::endl;
          exit(3);
        }
      } else if (strcmp(argv[1], "2nd") == 0) {
        publish_topic_name = "yaw_rate_offset_2nd";
        subscribe_topic_name = "heading_interpolate_2nd";

        try {
          YAML::Node conf = YAML::LoadFile(yaml_file);

          yaw_rate_offset_parameter_.imu_rate =
            conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
          yaw_rate_offset_parameter_.gnss_rate =
            conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
          yaw_rate_offset_parameter_.moving_judgment_threshold =
            conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
          yaw_rate_offset_parameter_.estimated_minimum_interval =
            conf["/**"]["ros__parameters"]["yaw_rate_offset"]["estimated_minimum_interval"]
              .as<double>();
          yaw_rate_offset_parameter_.estimated_maximum_interval =
            conf["/**"]["ros__parameters"]["yaw_rate_offset"]["2nd"]["estimated_maximum_interval"]
              .as<double>();
          yaw_rate_offset_parameter_.gnss_receiving_threshold =
            conf["/**"]["ros__parameters"]["yaw_rate_offset"]["gnss_receiving_threshold"]
              .as<double>();
          yaw_rate_offset_parameter_.outlier_threshold =
            conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

          std::cout << "imu_rate " << yaw_rate_offset_parameter_.imu_rate << std::endl;
          std::cout << "gnss_rate " << yaw_rate_offset_parameter_.gnss_rate << std::endl;
          std::cout << "moving_judgment_threshold "
                    << yaw_rate_offset_parameter_.moving_judgment_threshold << std::endl;
          std::cout << "estimated_minimum_interval "
                    << yaw_rate_offset_parameter_.estimated_minimum_interval << std::endl;
          std::cout << "estimated_maximum_interval "
                    << yaw_rate_offset_parameter_.estimated_maximum_interval << std::endl;
          std::cout << "gnss_receiving_threshold "
                    << yaw_rate_offset_parameter_.gnss_receiving_threshold << std::endl;
          std::cout << "outlier_threshold " << yaw_rate_offset_parameter_.outlier_threshold
                    << std::endl;
        } catch (YAML::Exception& e) {
          std::cerr << "\033[1;yaw_rate_offset_2nd Node YAML Error: " << e.msg << "\033[0m"
                    << std::endl;
          exit(3);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "No arguments");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "No arguments");
      rclcpp::shutdown();
    }

    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&YawRateOffsetNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&YawRateOffsetNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(&YawRateOffsetNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_heading_interpolate_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      subscribe_topic_name, 1000,
      std::bind(&YawRateOffsetNode::headingInterpolateCallback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&YawRateOffsetNode::imuCallback, this, std::placeholders::_1));
    pub_ =
      this->create_publisher<eagleye_msgs::msg::YawrateOffset>(publish_topic_name, rclcpp::QoS(10));
  }

private:
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::Heading heading_interpolate_;
  sensor_msgs::msg::Imu imu_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_;
  YawrateOffsetParameter yaw_rate_offset_parameter_;
  YawrateOffsetStatus yaw_rate_offset_status_ = {};
  bool is_first_heading_ = false;
  bool use_can_less_mode_ = false;
  double previous_yaw_rate_offset_ = 0.0;

  rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

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

  void headingInterpolateCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_ = *msg;
    if (is_first_heading_ == false && heading_interpolate_.status.enabled_status == true) {
      is_first_heading_ = true;
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (is_first_heading_ == false) return;
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    imu_ = *msg;
    yaw_rate_offset_.header = msg->header;
    yaw_rate_offset_estimate(
      velocity_, yaw_rate_offset_stop_, heading_interpolate_, imu_, yaw_rate_offset_parameter_,
      &yaw_rate_offset_status_, &yaw_rate_offset_);

    yaw_rate_offset_.status.is_abnormal = false;
    if (!std::isfinite(yaw_rate_offset_stop_.yaw_rate_offset)) {
      yaw_rate_offset_stop_.yaw_rate_offset = previous_yaw_rate_offset_;
      yaw_rate_offset_.status.is_abnormal = true;
      yaw_rate_offset_.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
    } else {
      previous_yaw_rate_offset_ = yaw_rate_offset_stop_.yaw_rate_offset;
    }

    pub_->publish(yaw_rate_offset_);
    yaw_rate_offset_.status.estimate_status = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawRateOffsetNode>(argc, argv));
  return 0;
}
