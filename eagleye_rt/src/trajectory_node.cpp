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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class TrajectoryNode : public rclcpp::Node
{
public:
  TrajectoryNode() : Node("eagleye_trajectory")
  {
    std::string subscribe_twist_topic_name = "vehicle/twist";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    double timer_update_rate = 10;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      trajectory_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      trajectory_parameter_.curve_judgment_threshold =
        conf["/**"]["ros__parameters"]["trajectory"]["curve_judgment_threshold"].as<double>();
      trajectory_parameter_.sensor_noise_velocity =
        conf["/**"]["ros__parameters"]["trajectory"]["sensor_noise_velocity"].as<double>();
      trajectory_parameter_.sensor_scale_noise_velocity =
        conf["/**"]["ros__parameters"]["trajectory"]["sensor_scale_noise_velocity"]
          .as<double>();
      trajectory_parameter_.sensor_noise_yaw_rate =
        conf["/**"]["ros__parameters"]["trajectory"]["sensor_noise_yaw_rate"].as<double>();
      trajectory_parameter_.sensor_bias_noise_yaw_rate =
        conf["/**"]["ros__parameters"]["trajectory"]["sensor_bias_noise_yaw_rate"].as<double>();
      timer_update_rate =
        conf["/**"]["ros__parameters"]["trajectory"]["timer_update_rate"].as<double>();

      std::cout << "use_can_less_mode " << use_can_less_mode_ << std::endl;
      std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;
      std::cout << "stop_judgment_threshold " << trajectory_parameter_.stop_judgment_threshold
                << std::endl;
      std::cout << "curve_judgment_threshold " << trajectory_parameter_.curve_judgment_threshold
                << std::endl;
      std::cout << "sensor_noise_velocity " << trajectory_parameter_.sensor_noise_velocity
                << std::endl;
      std::cout << "sensor_scale_noise_velocity "
                << trajectory_parameter_.sensor_scale_noise_velocity << std::endl;
      std::cout << "sensor_noise_yaw_rate " << trajectory_parameter_.sensor_noise_yaw_rate
                << std::endl;
      std::cout << "sensor_bias_noise_yaw_rate "
                << trajectory_parameter_.sensor_bias_noise_yaw_rate << std::endl;
      std::cout << "timer_update_rate " << timer_update_rate << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mtrajectory Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&TrajectoryNode::imuCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      subscribe_twist_topic_name, rclcpp::QoS(10),
      std::bind(&TrajectoryNode::velocityCallback, this, std::placeholders::_1));
    sub_correction_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&TrajectoryNode::correctionVelocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&TrajectoryNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_velocity_scale_factor_ =
      this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
        "velocity_scale_factor", rclcpp::QoS(10),
        std::bind(
          &TrajectoryNode::velocityScaleFactorCallback, this, std::placeholders::_1));
    sub_heading_interpolate_3rd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_3rd", rclcpp::QoS(10),
      std::bind(
        &TrajectoryNode::headingInterpolate3rdCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(&TrajectoryNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_2nd_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_2nd", rclcpp::QoS(10),
      std::bind(&TrajectoryNode::yawRateOffset2ndCallback, this, std::placeholders::_1));
    sub_pitching_ = this->create_subscription<eagleye_msgs::msg::Pitching>(
      "pitching", rclcpp::QoS(10),
      std::bind(&TrajectoryNode::pitchingCallback, this, std::placeholders::_1));

    pub_enu_vel_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000);
    pub_enu_relative_pos_ =
      this->create_publisher<eagleye_msgs::msg::Position>("enu_relative_pos", 1000);
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1000);
    pub_twist_with_covariance_ =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "twist_with_covariance", 1000);

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / timer_update_rate));
    timer_ = this->create_wall_timer(
      period_ns, std::bind(&TrajectoryNode::onTimer, this));
  }

private:
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  geometry_msgs::msg::TwistStamped correction_velocity_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;
  eagleye_msgs::msg::Pitching pitching_;

  geometry_msgs::msg::Vector3Stamped enu_vel_;
  eagleye_msgs::msg::Position enu_relative_pos_;
  geometry_msgs::msg::TwistStamped eagleye_twist_;
  geometry_msgs::msg::TwistWithCovarianceStamped eagleye_twist_with_covariance_;

  TrajectoryParameter trajectory_parameter_;
  TrajectoryStatus trajectory_status_;

  double th_deadlock_time_ = 1;
  double imu_time_last_ = 0;
  double velocity_time_last_ = 0;
  bool input_status_ = false;
  bool use_can_less_mode_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_enu_vel_;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_enu_relative_pos_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    pub_twist_with_covariance_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_correction_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr
    sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_3rd_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::Pitching>::SharedPtr sub_pitching_;

  void correctionVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    correction_velocity_ = *msg;
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

  void headingInterpolate3rdCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_3rd_ = *msg;
  }

  void yawRateOffsetStopCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop_ = *msg;
  }

  void yawRateOffset2ndCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd_ = *msg;
  }

  void pitchingCallback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
  {
    pitching_ = *msg;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void onTimer()
  {
    rclcpp::Time imu_clock(imu_.header.stamp);
    double imu_time = imu_clock.seconds();
    rclcpp::Time velocity_clock(velocity_.header.stamp);
    double velocity_time = velocity_clock.seconds();
    if (
      std::abs(imu_time - imu_time_last_) < th_deadlock_time_ &&
      std::abs(velocity_time - velocity_time_last_) < th_deadlock_time_ &&
      std::abs(velocity_time - imu_time) < th_deadlock_time_) {
      input_status_ = true;
    } else {
      input_status_ = false;
      RCLCPP_WARN(this->get_logger(), "Twist is missing the required input topics.");
    }

    if (imu_time != imu_time_last_) imu_time_last_ = imu_time;
    if (velocity_time != velocity_time_last_) velocity_time_last_ = velocity_time;
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
    if (input_status_) {
      enu_vel_.header = msg->header;
      enu_vel_.header.frame_id = "gnss";
      enu_relative_pos_.header = msg->header;
      enu_relative_pos_.header.frame_id = "base_link";
      eagleye_twist_.header = msg->header;
      eagleye_twist_.header.frame_id = "base_link";
      eagleye_twist_with_covariance_.header = msg->header;
      eagleye_twist_with_covariance_.header.frame_id = "base_link";
      trajectory3d_estimate(
        imu_, correction_velocity_, velocity_enable_status, heading_interpolate_3rd_,
        yaw_rate_offset_stop_, yaw_rate_offset_2nd_, pitching_, trajectory_parameter_,
        &trajectory_status_, &enu_vel_, &enu_relative_pos_, &eagleye_twist_,
        &eagleye_twist_with_covariance_);

      if (heading_interpolate_3rd_.status.enabled_status) {
        pub_enu_vel_->publish(enu_vel_);
        pub_enu_relative_pos_->publish(enu_relative_pos_);
      }
      pub_twist_->publish(eagleye_twist_);
      pub_twist_with_covariance_->publish(eagleye_twist_with_covariance_);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryNode>());
  return 0;
}
