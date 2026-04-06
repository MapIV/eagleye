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

class RtkHeadingNode : public rclcpp::Node
{
public:
  RtkHeadingNode(int argc, char** argv) : Node("eagleye_rtk_heading")
  {
    std::string subscribe_gga_topic_name = "gnss/gga";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      heading_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      heading_parameter_.gnss_rate =
        conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      heading_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      heading_parameter_.slow_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["slow_judgment_threshold"].as<double>();
      heading_parameter_.update_distance =
        conf["/**"]["ros__parameters"]["rtk_heading"]["update_distance"].as<double>();
      heading_parameter_.estimated_minimum_interval =
        conf["/**"]["ros__parameters"]["rtk_heading"]["estimated_minimum_interval"].as<double>();
      heading_parameter_.estimated_maximum_interval =
        conf["/**"]["ros__parameters"]["rtk_heading"]["estimated_maximum_interval"].as<double>();
      heading_parameter_.gnss_receiving_threshold =
        conf["/**"]["ros__parameters"]["rtk_heading"]["gnss_receiving_threshold"].as<double>();
      heading_parameter_.outlier_threshold =
        conf["/**"]["ros__parameters"]["rtk_heading"]["outlier_threshold"].as<double>();
      heading_parameter_.outlier_ratio_threshold =
        conf["/**"]["ros__parameters"]["rtk_heading"]["outlier_ratio_threshold"].as<double>();
      heading_parameter_.curve_judgment_threshold =
        conf["/**"]["ros__parameters"]["rtk_heading"]["curve_judgment_threshold"].as<double>();

      std::cout << "use_can_less_mode " << use_can_less_mode_ << std::endl;
      std::cout << "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;
      std::cout << "imu_rate " << heading_parameter_.imu_rate << std::endl;
      std::cout << "gnss_rate " << heading_parameter_.gnss_rate << std::endl;
      std::cout << "stop_judgment_threshold " << heading_parameter_.stop_judgment_threshold
                << std::endl;
      std::cout << "slow_judgment_threshold " << heading_parameter_.slow_judgment_threshold
                << std::endl;
      std::cout << "update_distance " << heading_parameter_.update_distance << std::endl;
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
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mrtk_heading Node YAML Error: " << e.msg << "\033[0m" << std::endl;
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

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&RtkHeadingNode::imuCallback, this, std::placeholders::_1));
    sub_gga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      subscribe_gga_topic_name, 1000,
      std::bind(&RtkHeadingNode::ggaCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&RtkHeadingNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&RtkHeadingNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", 1000,
      std::bind(&RtkHeadingNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      subscribe_topic_name, 1000,
      std::bind(&RtkHeadingNode::yawRateOffsetCallback, this, std::placeholders::_1));
    sub_slip_angle_ = this->create_subscription<eagleye_msgs::msg::SlipAngle>(
      "slip_angle", 1000,
      std::bind(&RtkHeadingNode::slipAngleCallback, this, std::placeholders::_1));
    sub_heading_interpolate_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      subscribe_topic_name2, 1000,
      std::bind(&RtkHeadingNode::headingInterpolateCallback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<eagleye_msgs::msg::Distance>(
      "distance", 1000,
      std::bind(&RtkHeadingNode::distanceCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, 1000);
  }

private:
  nmea_msgs::msg::Gpgga gga_;
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::Distance distance_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  eagleye_msgs::msg::Heading heading_interpolate_;
  eagleye_msgs::msg::Heading heading_;
  RtkHeadingParameter heading_parameter_;
  RtkHeadingStatus heading_status_ = {};
  bool use_can_less_mode_ = false;

  rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_;
  rclcpp::Subscription<eagleye_msgs::msg::SlipAngle>::SharedPtr sub_slip_angle_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void ggaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg) { gga_ = *msg; }

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

  void distanceCallback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    distance_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    imu_ = *msg;
    heading_.header = msg->header;
    heading_.header.frame_id = "base_link";
    rtk_heading_estimate(
      gga_, imu_, velocity_, distance_, yaw_rate_offset_stop_, yaw_rate_offset_, slip_angle_,
      heading_interpolate_, heading_parameter_, &heading_status_, &heading_);

    if (heading_.status.estimate_status == true) {
      pub_->publish(heading_);
    }
    heading_.status.estimate_status = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtkHeadingNode>(argc, argv));
  return 0;
}
