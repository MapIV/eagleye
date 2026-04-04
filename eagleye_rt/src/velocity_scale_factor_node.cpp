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
 * velocity_scale_factor.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class VelocityScaleFactorNode : public rclcpp::Node
{
public:
  VelocityScaleFactorNode() : Node("eagleye_velocity_scale_factor")
  {
    std::string subscribe_twist_topic_name = "vehicle/twist";
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
    std::string subscribe_rmc_topic_name = "gnss/rmc";

    double velocity_scale_factor_save_duration = 100.0;

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_gnss_mode_ = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();

      velocity_scale_factor_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      velocity_scale_factor_parameter_.gnss_rate =
        conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      velocity_scale_factor_parameter_.moving_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();

      velocity_scale_factor_parameter_.estimated_minimum_interval =
        conf["/**"]["ros__parameters"]["velocity_scale_factor"]["estimated_minimum_interval"]
          .as<double>();
      velocity_scale_factor_parameter_.estimated_maximum_interval =
        conf["/**"]["ros__parameters"]["velocity_scale_factor"]["estimated_maximum_interval"]
          .as<double>();
      velocity_scale_factor_parameter_.gnss_receiving_threshold =
        conf["/**"]["ros__parameters"]["velocity_scale_factor"]["gnss_receiving_threshold"]
          .as<double>();

      this->declare_parameter(
        "velocity_scale_factor_save_str", velocity_scale_factor_save_str_);
      this->declare_parameter(
        "velocity_scale_factor.save_velocity_scale_factor",
        velocity_scale_factor_parameter_.save_velocity_scale_factor);
      this->declare_parameter(
        "velocity_scale_factor.velocity_scale_factor_save_duration",
        velocity_scale_factor_save_duration);
      this->declare_parameter(
        "velocity_scale_factor.th_velocity_scale_factor_percent",
        th_velocity_scale_factor_percent_);

      this->get_parameter(
        "velocity_scale_factor_save_str", velocity_scale_factor_save_str_);
      this->get_parameter(
        "velocity_scale_factor.save_velocity_scale_factor",
        velocity_scale_factor_parameter_.save_velocity_scale_factor);
      this->get_parameter(
        "velocity_scale_factor.velocity_scale_factor_save_duration",
        velocity_scale_factor_save_duration);
      this->get_parameter(
        "velocity_scale_factor.th_velocity_scale_factor_percent",
        th_velocity_scale_factor_percent_);

      std::cout << "use_gnss_mode " << use_gnss_mode_ << std::endl;
      std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;
      std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name
                << std::endl;
      std::cout << "gnss_rate " << velocity_scale_factor_parameter_.gnss_rate << std::endl;
      std::cout << "moving_judgment_threshold "
                << velocity_scale_factor_parameter_.moving_judgment_threshold << std::endl;
      std::cout << "estimated_minimum_interval "
                << velocity_scale_factor_parameter_.estimated_minimum_interval << std::endl;
      std::cout << "estimated_maximum_interval "
                << velocity_scale_factor_parameter_.estimated_maximum_interval << std::endl;
      std::cout << "gnss_receiving_threshold "
                << velocity_scale_factor_parameter_.gnss_receiving_threshold << std::endl;
      std::cout << "velocity_scale_factor_save_str " << velocity_scale_factor_save_str_
                << std::endl;
      std::cout << "save_velocity_scale_factor "
                << velocity_scale_factor_parameter_.save_velocity_scale_factor << std::endl;
      std::cout << "velocity_scale_factor_save_duration " << velocity_scale_factor_save_duration
                << std::endl;
      std::cout << "th_velocity_scale_factor_percent " << th_velocity_scale_factor_percent_
                << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mvelocity_scale_factor Node YAML Error: " << e.msg << "\033[0m"
                << std::endl;
      exit(3);
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&VelocityScaleFactorNode::imuCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      subscribe_twist_topic_name, 1000,
      std::bind(&VelocityScaleFactorNode::velocityCallback, this, std::placeholders::_1));
    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      subscribe_rtklib_nav_topic_name, 1000,
      std::bind(&VelocityScaleFactorNode::rtklibNavCallback, this, std::placeholders::_1));
    sub_rmc_ = this->create_subscription<nmea_msgs::msg::Gprmc>(
      subscribe_rmc_topic_name, 1000,
      std::bind(&VelocityScaleFactorNode::rmcCallback, this, std::placeholders::_1));
    pub_correction_velocity_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10));
    pub_velocity_scale_factor_ = this->create_publisher<eagleye_msgs::msg::VelocityScaleFactor>(
      "velocity_scale_factor", rclcpp::QoS(10));

    if (velocity_scale_factor_parameter_.save_velocity_scale_factor) {
      const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(velocity_scale_factor_save_duration));
      timer_ = this->create_wall_timer(
        period_ns, std::bind(&VelocityScaleFactorNode::onTimer, this));
      loadVelocityScaleFactor(velocity_scale_factor_save_str_);
    }
  }

private:
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  nmea_msgs::msg::Gprmc nmea_rmc_;
  geometry_msgs::msg::TwistStamped velocity_;
  sensor_msgs::msg::Imu imu_;
  geometry_msgs::msg::TwistStamped correction_velocity_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;

  VelocityScaleFactorParameter velocity_scale_factor_parameter_;
  VelocityScaleFactorStatus velocity_scale_factor_status_;

  std::string use_gnss_mode_;
  bool is_first_move_ = false;
  std::string velocity_scale_factor_save_str_;
  double saved_vsf_estimater_number_ = 0;
  double saved_velocity_scale_factor_ = 1.0;
  double previous_velocity_scale_factor_ = 1.0;
  double th_velocity_scale_factor_percent_ = 20;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_correction_velocity_;
  rclcpp::Publisher<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr
    pub_velocity_scale_factor_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<nmea_msgs::msg::Gprmc>::SharedPtr sub_rmc_;

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_ = *msg;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;

    if (
      is_first_move_ == false &&
      msg->twist.linear.x > velocity_scale_factor_parameter_.moving_judgment_threshold) {
      is_first_move_ = true;
    }
  }

  void rmcCallback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg) { nmea_rmc_ = *msg; }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    double initial_velocity_scale_factor = saved_velocity_scale_factor_;

    imu_ = *msg;
    velocity_scale_factor_.header = msg->header;
    velocity_scale_factor_.header.frame_id = "base_link";

    correction_velocity_.header = msg->header;
    correction_velocity_.header.frame_id = "base_link";

    if (is_first_move_ == false) {
      velocity_scale_factor_.scale_factor = initial_velocity_scale_factor;
      correction_velocity_.twist = velocity_.twist;
      pub_correction_velocity_->publish(correction_velocity_);
      pub_velocity_scale_factor_->publish(velocity_scale_factor_);
      return;
    }

    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB") {
      velocity_scale_factor_estimate(
        rtklib_nav_, velocity_, velocity_scale_factor_parameter_,
        &velocity_scale_factor_status_, &correction_velocity_, &velocity_scale_factor_);
    } else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA") {
      velocity_scale_factor_estimate(
        nmea_rmc_, velocity_, velocity_scale_factor_parameter_, &velocity_scale_factor_status_,
        &correction_velocity_, &velocity_scale_factor_);
    }

    velocity_scale_factor_.status.is_abnormal = false;
    if (!std::isfinite(velocity_scale_factor_.scale_factor)) {
      correction_velocity_.twist.linear.x =
        velocity_.twist.linear.x * previous_velocity_scale_factor_;
      velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
      velocity_scale_factor_.status.is_abnormal = true;
      velocity_scale_factor_.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
    } else if (
      th_velocity_scale_factor_percent_ / 100 <
      std::abs(1.0 - velocity_scale_factor_.scale_factor)) {
      correction_velocity_.twist.linear.x =
        velocity_.twist.linear.x * previous_velocity_scale_factor_;
      velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
      velocity_scale_factor_.status.is_abnormal = true;
      velocity_scale_factor_.status.error_code = eagleye_msgs::msg::Status::TOO_LARGE_OR_SMALL;
    } else {
      previous_velocity_scale_factor_ = velocity_scale_factor_.scale_factor;
    }

    pub_correction_velocity_->publish(correction_velocity_);
    pub_velocity_scale_factor_->publish(velocity_scale_factor_);
  }

  void loadVelocityScaleFactor(std::string txt_path)
  {
    std::ifstream ifs(txt_path);
    if (!ifs) {
      std::cout << "Initial VelocityScaleFactor file not found!" << std::endl;
    } else {
      std::cout << "Loaded the saved velocity scale factor!" << std::endl;
      int count = 0;
      std::string row;
      while (getline(ifs, row)) {
        if (count == 1) {
          saved_vsf_estimater_number_ = std::stod(row);
          std::cout << "saved_vsf_estimater_number " << saved_vsf_estimater_number_ << std::endl;
        }
        if (count == 3) {
          saved_velocity_scale_factor_ = std::stod(row);
          velocity_scale_factor_status_.estimate_start_status = true;
          velocity_scale_factor_status_.velocity_scale_factor_last =
            saved_velocity_scale_factor_;
          velocity_scale_factor_.status.enabled_status = true;
          velocity_scale_factor_.scale_factor = saved_velocity_scale_factor_;
          std::cout << "saved_velocity_scale_factor " << saved_velocity_scale_factor_
                    << std::endl;
        }
        count++;
      }
    }
    ifs.close();
  }

  void onTimer()
  {
    if (
      !velocity_scale_factor_.status.enabled_status &&
      saved_vsf_estimater_number_ >= velocity_scale_factor_status_.estimated_number) {
      std::ofstream csv_file(velocity_scale_factor_save_str_);
      return;
    }

    std::ofstream csv_file(velocity_scale_factor_save_str_);
    csv_file << "estimated_number";
    csv_file << "\n";
    csv_file << velocity_scale_factor_status_.estimated_number;
    csv_file << "\n";
    csv_file << "velocity_scale_factor";
    csv_file << "\n";
    csv_file << velocity_scale_factor_status_.velocity_scale_factor_last;
    csv_file << "\n";
    csv_file.close();

    saved_vsf_estimater_number_ = velocity_scale_factor_status_.estimated_number;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityScaleFactorNode>());
  return 0;
}
