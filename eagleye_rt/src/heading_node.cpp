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
 * heading.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class HeadingNode : public rclcpp::Node
{
public:
  HeadingNode(int argc, char** argv) : Node("eagleye_heading")
  {
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
    std::string subscribe_rmc_topic_name = "gnss/rmc";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    this->declare_parameter("use_multi_antenna_mode", use_multi_antenna_mode_);
    this->get_parameter("use_multi_antenna_mode", use_multi_antenna_mode_);
    std::cout << "yaml_file: " << yaml_file << std::endl;
    std::cout << "use_multi_antenna_mode: " << use_multi_antenna_mode_ << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_gnss_mode_ = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();
      heading_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      heading_parameter_.gnss_rate =
        conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      heading_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      heading_parameter_.moving_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
      heading_parameter_.estimated_minimum_interval =
        conf["/**"]["ros__parameters"]["heading"]["estimated_minimum_interval"].as<double>();
      heading_parameter_.estimated_maximum_interval =
        conf["/**"]["ros__parameters"]["heading"]["estimated_maximum_interval"].as<double>();
      heading_parameter_.gnss_receiving_threshold =
        conf["/**"]["ros__parameters"]["heading"]["gnss_receiving_threshold"].as<double>();
      heading_parameter_.outlier_threshold =
        conf["/**"]["ros__parameters"]["heading"]["outlier_threshold"].as<double>();
      heading_parameter_.outlier_ratio_threshold =
        conf["/**"]["ros__parameters"]["heading"]["outlier_ratio_threshold"].as<double>();
      heading_parameter_.curve_judgment_threshold =
        conf["/**"]["ros__parameters"]["heading"]["curve_judgment_threshold"].as<double>();
      heading_parameter_.init_STD =
        conf["/**"]["ros__parameters"]["heading"]["init_STD"].as<double>();
      skip_static_initialization_ =
        conf["/**"]["ros__parameters"]["heading"]["skip_static_initialization"].as<bool>();
      yaw_rate_offset_stop_in_skip_mode_ =
        conf["/**"]["ros__parameters"]["heading"]["yaw_rate_offset_stop_in_skip_mode"].as<double>();

      std::cout << "use_gnss_mode " << use_gnss_mode_ << std::endl;
      std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name
                << std::endl;
      std::cout << "subscribe_rmc_topic_name " << subscribe_rmc_topic_name << std::endl;
      std::cout << "imu_rate " << heading_parameter_.imu_rate << std::endl;
      std::cout << "gnss_rate " << heading_parameter_.gnss_rate << std::endl;
      std::cout << "stop_judgment_threshold " << heading_parameter_.stop_judgment_threshold
                << std::endl;
      std::cout << "moving_judgment_threshold " << heading_parameter_.moving_judgment_threshold
                << std::endl;
      std::cout << "estimated_minimum_interval " << heading_parameter_.estimated_minimum_interval
                << std::endl;
      std::cout << "estimated_maximum_interval " << heading_parameter_.estimated_maximum_interval
                << std::endl;
      std::cout << "gnss_receiving_threshold " << heading_parameter_.gnss_receiving_threshold
                << std::endl;
      std::cout << "outlier_threshold " << heading_parameter_.outlier_threshold << std::endl;
      std::cout << "outlier_ratio_threshold " << heading_parameter_.outlier_ratio_threshold
                << std::endl;
      std::cout << "curve_judgment_threshold " << heading_parameter_.curve_judgment_threshold
                << std::endl;
      std::cout << "init_STD " << heading_parameter_.init_STD << std::endl;
      std::cout << "skip_static_initialization " << skip_static_initialization_ << std::endl;
      std::cout << "yaw_rate_offset_stop_in_skip_mode " << yaw_rate_offset_stop_in_skip_mode_
                << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mheading Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    std::string publish_topic_name = "/publish_topic_name/invalid";
    std::string subscribe_topic_name = "/subscribe_topic_name/invalid";
    std::string subscribe_topic_name2 = "/subscribe_topic_name2/invalid";

    if (argc > 2) {
      if (strcmp(argv[1], "1st") == 0) {
        publish_topic_name = "heading_1st";
        subscribe_topic_name = "yaw_rate_offset_stop";
        subscribe_topic_name2 = "heading_interpolate_1st";
      } else if (strcmp(argv[1], "2nd") == 0) {
        publish_topic_name = "heading_2nd";
        subscribe_topic_name = "yaw_rate_offset_1st";
        subscribe_topic_name2 = "heading_interpolate_2nd";
      } else if (strcmp(argv[1], "3rd") == 0) {
        publish_topic_name = "heading_3rd";
        subscribe_topic_name = "yaw_rate_offset_2nd";
        subscribe_topic_name2 = "heading_interpolate_3rd";
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid argument");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "No arguments");
      rclcpp::shutdown();
    }

    if (use_multi_antenna_mode_) {
      is_first_correction_velocity_ = true;
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&HeadingNode::imuCallback, this, std::placeholders::_1));
    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      subscribe_rtklib_nav_topic_name, 1000,
      std::bind(&HeadingNode::rtklibNavCallback, this, std::placeholders::_1));
    sub_rmc_ = this->create_subscription<nmea_msgs::msg::Gprmc>(
      subscribe_rmc_topic_name, 1000,
      std::bind(&HeadingNode::rmcCallback, this, std::placeholders::_1));
    sub_gnss_compass_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "gnss_compass_pose", 1000,
      std::bind(&HeadingNode::poseCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&HeadingNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&HeadingNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(&HeadingNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      subscribe_topic_name, 1000,
      std::bind(&HeadingNode::yawRateOffsetCallback, this, std::placeholders::_1));
    sub_slip_angle_ = this->create_subscription<eagleye_msgs::msg::SlipAngle>(
      "slip_angle", rclcpp::QoS(10),
      std::bind(&HeadingNode::slipAngleCallback, this, std::placeholders::_1));
    sub_heading_interpolate_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      subscribe_topic_name2, 1000,
      std::bind(&HeadingNode::headingInterpolateCallback, this, std::placeholders::_1));
    pub_ =
      this->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, rclcpp::QoS(10));
  }

private:
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  nmea_msgs::msg::Gprmc nmea_rmc_;
  eagleye_msgs::msg::Heading multi_antenna_heading_;
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  eagleye_msgs::msg::Heading heading_interpolate_;
  eagleye_msgs::msg::Heading heading_;
  HeadingParameter heading_parameter_;
  HeadingStatus heading_status_;
  std::string use_gnss_mode_;
  bool use_can_less_mode_ = false;
  bool use_multi_antenna_mode_ = false;
  bool is_first_correction_velocity_ = false;
  bool skip_static_initialization_ = false;
  double yaw_rate_offset_stop_in_skip_mode_ = 0.0;

  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<nmea_msgs::msg::Gprmc>::SharedPtr sub_rmc_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_gnss_compass_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_;
  rclcpp::Subscription<eagleye_msgs::msg::SlipAngle>::SharedPtr sub_slip_angle_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_;

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_ = *msg;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
    if (
      is_first_correction_velocity_ == false &&
      msg->twist.linear.x > heading_parameter_.moving_judgment_threshold) {
      is_first_correction_velocity_ = true;
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    tf2::Quaternion orientation;
    tf2::fromMsg(msg->pose.orientation, orientation);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    double heading = -yaw + (90 * M_PI / 180);

    multi_antenna_heading_.header = msg->header;
    multi_antenna_heading_.heading_angle = heading;
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

  void slipAngleCallback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
  {
    slip_angle_ = *msg;
  }

  void headingInterpolateCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_ = *msg;
  }

  void rmcCallback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg) { nmea_rmc_ = *msg; }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!is_first_correction_velocity_) {
      RCLCPP_WARN(this->get_logger(), "is_first_correction_velocity is false.");
      return;
    }
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) {
      RCLCPP_WARN(this->get_logger(), "velocity_status is not enabled.");
      return;
    }
    if (!yaw_rate_offset_stop_.status.enabled_status) {
      if (skip_static_initialization_) {
        yaw_rate_offset_stop_.yaw_rate_offset = yaw_rate_offset_stop_in_skip_mode_;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Heading estimation is not started because the stop calibration is not yet completed.");
        return;
      }
    }

    imu_ = *msg;
    heading_.header = msg->header;
    heading_.header.frame_id = "base_link";
    bool use_rtklib_mode = use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB";
    bool use_nmea_mode = use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA";
    if (use_rtklib_mode && !use_multi_antenna_mode_)
      heading_estimate(
        rtklib_nav_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_, slip_angle_,
        heading_interpolate_, heading_parameter_, &heading_status_, &heading_);
    else if (use_nmea_mode && !use_multi_antenna_mode_)
      heading_estimate(
        nmea_rmc_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_, slip_angle_,
        heading_interpolate_, heading_parameter_, &heading_status_, &heading_);
    else if (use_multi_antenna_mode_)
      heading_estimate(
        multi_antenna_heading_, imu_, velocity_, yaw_rate_offset_stop_, yaw_rate_offset_,
        slip_angle_, heading_interpolate_, heading_parameter_, &heading_status_, &heading_);

    if (heading_.status.estimate_status == true || use_multi_antenna_mode_) {
      pub_->publish(heading_);
    }
    heading_.status.estimate_status = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingNode>(argc, argv));
  return 0;
}
