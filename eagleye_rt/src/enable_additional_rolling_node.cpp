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
 * enable_additional_rolling_node.cpp
 * Author MapIV Hoda
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class EnableAdditionalRollingNode : public rclcpp::Node
{
public:
  EnableAdditionalRollingNode() : Node("eagleye_enable_additional_rolling")
  {
    std::string subscribe_localization_pose_topic_name;

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      subscribe_localization_pose_topic_name =
        conf["/**"]["ros__parameters"]["localization_pose_topic"].as<std::string>();

      rolling_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      rolling_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      rolling_parameter_.update_distance =
        conf["/**"]["ros__parameters"]["enable_additional_rolling"]["update_distance"]
          .as<double>();
      rolling_parameter_.moving_average_time =
        conf["/**"]["ros__parameters"]["enable_additional_rolling"]["moving_average_time"]
          .as<double>();
      rolling_parameter_.sync_judgment_threshold =
        conf["/**"]["ros__parameters"]["enable_additional_rolling"]["sync_judgment_threshold"]
          .as<double>();
      rolling_parameter_.sync_search_period =
        conf["/**"]["ros__parameters"]["enable_additional_rolling"]["sync_search_period"]
          .as<double>();

      std::cout << "subscribe_localization_pose_topic_name "
                << subscribe_localization_pose_topic_name << std::endl;
      std::cout << "imu_rate " << rolling_parameter_.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold " << rolling_parameter_.stop_judgment_threshold
                << std::endl;
      std::cout << "update_distance " << rolling_parameter_.update_distance << std::endl;
      std::cout << "moving_average_time " << rolling_parameter_.moving_average_time << std::endl;
      std::cout << "sync_judgment_threshold " << rolling_parameter_.sync_judgment_threshold
                << std::endl;
      std::cout << "sync_search_period " << rolling_parameter_.sync_search_period << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31menable_additional_rolling Node YAML Error: " << e.msg << "\033[0m"
                << std::endl;
      exit(3);
    }

    sub_velocity_scale_factor_ =
      this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
        "velocity_scale_factor", 1000,
        std::bind(
          &EnableAdditionalRollingNode::velocityScaleFactorCallback, this,
          std::placeholders::_1));
    sub_yaw_rate_offset_2nd_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_2nd", 1000,
      std::bind(
        &EnableAdditionalRollingNode::yawRateOffset2ndCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", 1000,
      std::bind(
        &EnableAdditionalRollingNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<eagleye_msgs::msg::Distance>(
      "distance", 1000,
      std::bind(&EnableAdditionalRollingNode::distanceCallback, this, std::placeholders::_1));
    sub_localization_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      subscribe_localization_pose_topic_name, 1000,
      std::bind(
        &EnableAdditionalRollingNode::localizationPoseCallback, this, std::placeholders::_1));
    sub_angular_velocity_offset_stop_ =
      this->create_subscription<eagleye_msgs::msg::AngularVelocityOffset>(
        "angular_velocity_offset_stop", 1000,
        std::bind(
          &EnableAdditionalRollingNode::angularVelocityOffsetStopCallback, this,
          std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&EnableAdditionalRollingNode::imuCallback, this, std::placeholders::_1));

    pub_acc_y_offset_ = this->create_publisher<eagleye_msgs::msg::AccYOffset>(
      "acc_y_offset_additional_rolling", 1000);
    pub_rolling_ =
      this->create_publisher<eagleye_msgs::msg::Rolling>("enable_additional_rolling", 1000);
  }

private:
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::Distance distance_;
  geometry_msgs::msg::PoseStamped localization_pose_;
  eagleye_msgs::msg::AngularVelocityOffset angular_velocity_offset_stop_;
  sensor_msgs::msg::Imu imu_;

  eagleye_msgs::msg::Rolling rolling_angle_;
  eagleye_msgs::msg::AccYOffset acc_y_offset_;

  EnableAdditionalRollingParameter rolling_parameter_;
  EnableAdditionalRollingStatus rolling_status_ = {};

  bool use_can_less_mode_ = false;

  rclcpp::Publisher<eagleye_msgs::msg::AccYOffset>::SharedPtr pub_acc_y_offset_;
  rclcpp::Publisher<eagleye_msgs::msg::Rolling>::SharedPtr pub_rolling_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr
    sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_localization_pose_;
  rclcpp::Subscription<eagleye_msgs::msg::AngularVelocityOffset>::SharedPtr
    sub_angular_velocity_offset_stop_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  void velocityScaleFactorCallback(
    const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
  {
    velocity_scale_factor_ = *msg;
  }

  void yawRateOffset2ndCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd_ = *msg;
  }

  void yawRateOffsetStopCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop_ = *msg;
  }

  void distanceCallback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    distance_ = *msg;
  }

  void localizationPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    localization_pose_ = *msg;
  }

  void angularVelocityOffsetStopCallback(
    const eagleye_msgs::msg::AngularVelocityOffset::ConstSharedPtr msg)
  {
    angular_velocity_offset_stop_ = *msg;
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
    acc_y_offset_.header = msg->header;
    acc_y_offset_.header.frame_id = "imu";
    rolling_angle_.header = msg->header;
    rolling_angle_.header.frame_id = "base_link";
    enable_additional_rolling_estimate(
      velocity_, velocity_enable_status, yaw_rate_offset_2nd_, yaw_rate_offset_stop_, distance_,
      imu_, localization_pose_, angular_velocity_offset_stop_, rolling_parameter_,
      &rolling_status_, &rolling_angle_, &acc_y_offset_);
    pub_acc_y_offset_->publish(acc_y_offset_);
    pub_rolling_->publish(rolling_angle_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnableAdditionalRollingNode>());
  return 0;
}
