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
 * height_node.cpp
 * Author MapIV  Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class HeightNode : public rclcpp::Node
{
public:
  HeightNode() : Node("eagleye_height")
  {
    std::string subscribe_gga_topic_name = "gnss/gga";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      height_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      height_parameter_.gnss_rate =
        conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      height_parameter_.moving_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
      height_parameter_.estimated_minimum_interval =
        conf["/**"]["ros__parameters"]["height"]["estimated_minimum_interval"].as<double>();
      height_parameter_.estimated_maximum_interval =
        conf["/**"]["ros__parameters"]["height"]["estimated_maximum_interval"].as<double>();
      height_parameter_.update_distance =
        conf["/**"]["ros__parameters"]["height"]["update_distance"].as<double>();
      height_parameter_.gnss_receiving_threshold =
        conf["/**"]["ros__parameters"]["height"]["gnss_receiving_threshold"].as<double>();
      height_parameter_.outlier_threshold =
        conf["/**"]["ros__parameters"]["height"]["outlier_threshold"].as<double>();
      height_parameter_.outlier_ratio_threshold =
        conf["/**"]["ros__parameters"]["height"]["outlier_ratio_threshold"].as<double>();
      height_parameter_.moving_average_time =
        conf["/**"]["ros__parameters"]["height"]["moving_average_time"].as<double>();

      std::cout << "imu_rate " << height_parameter_.imu_rate << std::endl;
      std::cout << "gnss_rate " << height_parameter_.gnss_rate << std::endl;
      std::cout << "moving_judgment_threshold " << height_parameter_.moving_judgment_threshold
                << std::endl;
      std::cout << "estimated_minimum_interval " << height_parameter_.estimated_minimum_interval
                << std::endl;
      std::cout << "estimated_maximum_interval " << height_parameter_.estimated_maximum_interval
                << std::endl;
      std::cout << "update_distance " << height_parameter_.update_distance << std::endl;
      std::cout << "gnss_receiving_threshold " << height_parameter_.gnss_receiving_threshold
                << std::endl;
      std::cout << "outlier_threshold " << height_parameter_.outlier_threshold << std::endl;
      std::cout << "outlier_ratio_threshold " << height_parameter_.outlier_ratio_threshold
                << std::endl;
      std::cout << "moving_average_time " << height_parameter_.moving_average_time << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mheight Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&HeightNode::imuCallback, this, std::placeholders::_1));
    sub_gga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      subscribe_gga_topic_name, 1000,
      std::bind(&HeightNode::ggaCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&HeightNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&HeightNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<eagleye_msgs::msg::Distance>(
      "distance", rclcpp::QoS(10),
      std::bind(&HeightNode::distanceCallback, this, std::placeholders::_1));

    pub_height_ = this->create_publisher<eagleye_msgs::msg::Height>("height", 1000);
    pub_pitching_ = this->create_publisher<eagleye_msgs::msg::Pitching>("pitching", 1000);
    pub_acc_x_offset_ =
      this->create_publisher<eagleye_msgs::msg::AccXOffset>("acc_x_offset", 1000);
    pub_acc_x_scale_factor_ =
      this->create_publisher<eagleye_msgs::msg::AccXScaleFactor>("acc_x_scale_factor", 1000);
    pub_gga_ =
      this->create_publisher<nmea_msgs::msg::Gpgga>("navsat/reliability_gga", 1000);
  }

private:
  sensor_msgs::msg::Imu imu_;
  nmea_msgs::msg::Gpgga gga_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::Distance distance_;
  eagleye_msgs::msg::Height height_;
  eagleye_msgs::msg::Pitching pitching_;
  eagleye_msgs::msg::AccXOffset acc_x_offset_;
  eagleye_msgs::msg::AccXScaleFactor acc_x_scale_factor_;
  HeightParameter height_parameter_;
  HeightStatus height_status_ = {};
  bool use_can_less_mode_ = false;

  rclcpp::Publisher<eagleye_msgs::msg::Height>::SharedPtr pub_height_;
  rclcpp::Publisher<eagleye_msgs::msg::Pitching>::SharedPtr pub_pitching_;
  rclcpp::Publisher<eagleye_msgs::msg::AccXOffset>::SharedPtr pub_acc_x_offset_;
  rclcpp::Publisher<eagleye_msgs::msg::AccXScaleFactor>::SharedPtr pub_acc_x_scale_factor_;
  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr pub_gga_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;

  void ggaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg) { gga_ = *msg; }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void velocityStatusCallback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status_ = *msg;
  }

  void distanceCallback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    distance_ = *msg;
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    imu_ = *msg;
    height_.header = msg->header;
    height_.header.frame_id = "base_link";
    pitching_.header = msg->header;
    pitching_.header.frame_id = "base_link";
    acc_x_offset_.header = msg->header;
    acc_x_scale_factor_.header = msg->header;
    pitching_estimate(
      imu_, gga_, velocity_, distance_, height_parameter_, &height_status_, &height_, &pitching_,
      &acc_x_offset_, &acc_x_scale_factor_);
    pub_height_->publish(height_);
    pub_pitching_->publish(pitching_);
    pub_acc_x_offset_->publish(acc_x_offset_);
    pub_acc_x_scale_factor_->publish(acc_x_scale_factor_);

    if (height_status_.flag_reliability == true) {
      pub_gga_->publish(gga_);
    }

    height_status_.flag_reliability = false;
    height_.status.estimate_status = false;
    pitching_.status.estimate_status = false;
    acc_x_offset_.status.estimate_status = false;
    acc_x_scale_factor_.status.estimate_status = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeightNode>());
  return 0;
}
