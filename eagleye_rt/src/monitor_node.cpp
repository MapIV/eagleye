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
 * monitor.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode() : Node("eagleye_monitor")
  {
    std::string subscribe_twist_topic_name = "vehicle/twist";
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
    std::string subscribe_gga_topic_name = "gnss/gga";
    std::string comparison_twist_topic_name = "/calculated_twist";

    this->declare_parameter("rtklib_nav_topic", subscribe_rtklib_nav_topic_name);
    this->declare_parameter("gga_topic", subscribe_gga_topic_name);
    this->declare_parameter("monitor.comparison_twist_topic", comparison_twist_topic_name);
    this->declare_parameter("monitor.print_status", print_status_);
    this->declare_parameter("monitor.log_output_status", log_output_status_);
    this->declare_parameter("monitor.use_compare_yaw_rate", use_compare_yaw_rate_);
    this->declare_parameter("monitor.th_diff_rad_per_sec", th_diff_rad_per_sec_);
    this->declare_parameter(
      "monitor.th_num_continuous_abnormal_yaw_rate", th_num_continuous_abnormal_yaw_rate_);

    this->get_parameter("rtklib_nav_topic", subscribe_rtklib_nav_topic_name);
    this->get_parameter("gga_topic", subscribe_gga_topic_name);
    this->get_parameter("monitor.comparison_twist_topic", comparison_twist_topic_name);
    this->get_parameter("monitor.print_status", print_status_);
    this->get_parameter("monitor.log_output_status", log_output_status_);
    this->get_parameter("monitor.use_compare_yaw_rate", use_compare_yaw_rate_);
    this->get_parameter("monitor.th_diff_rad_per_sec", th_diff_rad_per_sec_);
    this->get_parameter(
      "monitor.th_num_continuous_abnormal_yaw_rate", th_num_continuous_abnormal_yaw_rate_);

    std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name
              << std::endl;
    std::cout << "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;
    std::cout << "print_status " << print_status_ << std::endl;
    std::cout << "log_output_status " << log_output_status_ << std::endl;
    std::cout << "use_compare_yaw_rate " << use_compare_yaw_rate_ << std::endl;
    if (use_compare_yaw_rate_) {
      std::cout << "comparison_twist_topic_name " << comparison_twist_topic_name << std::endl;
      std::cout << "th_diff_rad_per_sec " << th_diff_rad_per_sec_ << std::endl;
      std::cout << "th_num_continuous_abnormal_yaw_rate "
                << th_num_continuous_abnormal_yaw_rate_ << std::endl;
    }

    double update_time = 1.0 / update_rate_;
    updater_ = std::make_shared<diagnostic_updater::Updater>(this, update_time);

    updater_->setHardwareID("eagleye_topic_checker");
    updater_->add(
      "eagleye_input_imu",
      std::bind(&MonitorNode::imuTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_input_rtklib_nav",
      std::bind(&MonitorNode::rtklibNavTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_input_navsat_gga",
      std::bind(&MonitorNode::navsatGgaTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_input_velocity",
      std::bind(&MonitorNode::velocityTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_velocity_scale_factor",
      std::bind(&MonitorNode::velocityScaleFactorTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_distance",
      std::bind(&MonitorNode::distanceTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_heading_1st",
      std::bind(&MonitorNode::heading1stTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_heading_interpolate_1st",
      std::bind(
        &MonitorNode::headingInterpolate1stTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_heading_2nd",
      std::bind(&MonitorNode::heading2ndTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_heading_interpolate_2nd",
      std::bind(
        &MonitorNode::headingInterpolate2ndTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_heading_3rd",
      std::bind(&MonitorNode::heading3rdTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_heading_interpolate_3rd",
      std::bind(
        &MonitorNode::headingInterpolate3rdTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_yaw_rate_offset_stop",
      std::bind(&MonitorNode::yawRateOffsetStopTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_yaw_rate_offset_1st",
      std::bind(&MonitorNode::yawRateOffset1stTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_yaw_rate_offset_2nd",
      std::bind(&MonitorNode::yawRateOffset2ndTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_slip_angle",
      std::bind(&MonitorNode::slipAngleTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_enu_vel",
      std::bind(&MonitorNode::enuVelTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_height",
      std::bind(&MonitorNode::heightTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_pitching",
      std::bind(&MonitorNode::pitchingTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_enu_absolute_pos",
      std::bind(&MonitorNode::enuAbsolutePosTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_enu_absolute_pos_interpolate",
      std::bind(
        &MonitorNode::enuAbsolutePosInterpolateTopicChecker, this, std::placeholders::_1));
    updater_->add(
      "eagleye_twist",
      std::bind(&MonitorNode::twistTopicChecker, this, std::placeholders::_1));
    if (use_compare_yaw_rate_) {
      updater_->add(
        "eagleye_imu_comparison",
        std::bind(&MonitorNode::imuComparisonChecker, this, std::placeholders::_1));
    }

    time_t time_;
    time_ = time(NULL);
    std::stringstream time_ss;
    time_ss << time_;
    std::string time_str = time_ss.str();
    output_log_dir_ = ament_index_cpp::get_package_share_directory("eagleye_rt") +
                      "/log/eagleye_log_" + time_str + ".csv";
    if (log_output_status_) std::cout << output_log_dir_ << std::endl;

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_tf_converted", 1000,
      std::bind(&MonitorNode::imuCallback, this, std::placeholders::_1));
    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      subscribe_rtklib_nav_topic_name, 1000,
      std::bind(&MonitorNode::rtklibNavCallback, this, std::placeholders::_1));
    sub_rtklib_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "rtklib/fix", rclcpp::QoS(10),
      std::bind(&MonitorNode::rtklibFixCallback, this, std::placeholders::_1));
    sub_gga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      subscribe_gga_topic_name, 1000,
      std::bind(&MonitorNode::navsatfixGgaCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      subscribe_twist_topic_name, 1000,
      std::bind(&MonitorNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_scale_factor_ =
      this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
        "velocity_scale_factor", rclcpp::QoS(10),
        std::bind(&MonitorNode::velocityScaleFactorCallback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<eagleye_msgs::msg::Distance>(
      "distance", rclcpp::QoS(10),
      std::bind(&MonitorNode::distanceCallback, this, std::placeholders::_1));
    sub_heading_1st_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_1st", rclcpp::QoS(10),
      std::bind(&MonitorNode::heading1stCallback, this, std::placeholders::_1));
    sub_heading_interpolate_1st_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_1st", rclcpp::QoS(10),
      std::bind(&MonitorNode::headingInterpolate1stCallback, this, std::placeholders::_1));
    sub_heading_2nd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_2nd", rclcpp::QoS(10),
      std::bind(&MonitorNode::heading2ndCallback, this, std::placeholders::_1));
    sub_heading_interpolate_2nd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_2nd", rclcpp::QoS(10),
      std::bind(&MonitorNode::headingInterpolate2ndCallback, this, std::placeholders::_1));
    sub_heading_3rd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_3rd", rclcpp::QoS(10),
      std::bind(&MonitorNode::heading3rdCallback, this, std::placeholders::_1));
    sub_heading_interpolate_3rd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_3rd", rclcpp::QoS(10),
      std::bind(&MonitorNode::headingInterpolate3rdCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_stop_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_stop", rclcpp::QoS(10),
      std::bind(&MonitorNode::yawRateOffsetStopCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_1st_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_1st", rclcpp::QoS(10),
      std::bind(&MonitorNode::yawRateOffset1stCallback, this, std::placeholders::_1));
    sub_yaw_rate_offset_2nd_ = this->create_subscription<eagleye_msgs::msg::YawrateOffset>(
      "yaw_rate_offset_2nd", rclcpp::QoS(10),
      std::bind(&MonitorNode::yawRateOffset2ndCallback, this, std::placeholders::_1));
    sub_slip_angle_ = this->create_subscription<eagleye_msgs::msg::SlipAngle>(
      "slip_angle", rclcpp::QoS(10),
      std::bind(&MonitorNode::slipAngleCallback, this, std::placeholders::_1));
    sub_enu_relative_pos_ = this->create_subscription<eagleye_msgs::msg::Position>(
      "enu_relative_pos", rclcpp::QoS(10),
      std::bind(&MonitorNode::enuRelativePosCallback, this, std::placeholders::_1));
    sub_enu_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "enu_vel", rclcpp::QoS(10),
      std::bind(&MonitorNode::enuVelCallback, this, std::placeholders::_1));
    sub_height_ = this->create_subscription<eagleye_msgs::msg::Height>(
      "height", rclcpp::QoS(10),
      std::bind(&MonitorNode::heightCallback, this, std::placeholders::_1));
    sub_pitching_ = this->create_subscription<eagleye_msgs::msg::Pitching>(
      "pitching", rclcpp::QoS(10),
      std::bind(&MonitorNode::pitchingCallback, this, std::placeholders::_1));
    sub_enu_absolute_pos_ = this->create_subscription<eagleye_msgs::msg::Position>(
      "enu_absolute_pos", rclcpp::QoS(10),
      std::bind(&MonitorNode::enuAbsolutePosCallback, this, std::placeholders::_1));
    sub_enu_absolute_pos_interpolate_ = this->create_subscription<eagleye_msgs::msg::Position>(
      "enu_absolute_pos_interpolate", rclcpp::QoS(10),
      std::bind(
        &MonitorNode::enuAbsolutePosInterpolateCallback, this, std::placeholders::_1));
    sub_eagleye_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", rclcpp::QoS(10),
      std::bind(&MonitorNode::eagleyeFixCallback, this, std::placeholders::_1));
    sub_eagleye_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "twist", rclcpp::QoS(10),
      std::bind(&MonitorNode::eagleyeTwistCallback, this, std::placeholders::_1));
    sub_rolling_ = this->create_subscription<eagleye_msgs::msg::Rolling>(
      "rolling", rclcpp::QoS(10),
      std::bind(&MonitorNode::rollingCallback, this, std::placeholders::_1));
    sub_comparison_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      comparison_twist_topic_name, 1000,
      std::bind(&MonitorNode::comparisonVelocityCallback, this, std::placeholders::_1));
    sub_correction_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", 1000,
      std::bind(&MonitorNode::correctionVelocityCallback, this, std::placeholders::_1));
  }

private:
  sensor_msgs::msg::Imu imu_;
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  sensor_msgs::msg::NavSatFix rtklib_fix_;
  nmea_msgs::msg::Gpgga gga_;
  geometry_msgs::msg::TwistStamped velocity_;
  geometry_msgs::msg::TwistStamped correction_velocity_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::Distance distance_;
  eagleye_msgs::msg::Heading heading_1st_;
  eagleye_msgs::msg::Heading heading_interpolate_1st_;
  eagleye_msgs::msg::Heading heading_2nd_;
  eagleye_msgs::msg::Heading heading_interpolate_2nd_;
  eagleye_msgs::msg::Heading heading_3rd_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_1st_;
  eagleye_msgs::msg::YawrateOffset yaw_rate_offset_2nd_;
  eagleye_msgs::msg::SlipAngle slip_angle_;
  eagleye_msgs::msg::Height height_;
  eagleye_msgs::msg::Pitching pitching_;
  eagleye_msgs::msg::Rolling rolling_;
  eagleye_msgs::msg::Position enu_relative_pos_;
  geometry_msgs::msg::Vector3Stamped enu_vel_;
  eagleye_msgs::msg::Position enu_absolute_pos_;
  eagleye_msgs::msg::Position enu_absolute_pos_interpolate_;
  sensor_msgs::msg::NavSatFix eagleye_fix_;
  geometry_msgs::msg::TwistStamped eagleye_twist_;

  geometry_msgs::msg::TwistStamped::ConstSharedPtr comparison_velocity_ptr_;
  sensor_msgs::msg::Imu corrected_imu_;

  bool gga_sub_status_ = false;
  bool print_status_ = false;
  bool log_output_status_ = false;
  bool log_header_make_ = false;
  std::string output_log_dir_;

  double imu_time_last_ = 0;
  double rtklib_nav_time_last_ = 0;
  double navsat_gga_time_last_ = 0;
  double velocity_time_last_ = 0;
  double velocity_scale_factor_time_last_ = 0;
  double distance_time_last_ = 0;
  double heading_1st_time_last_ = 0;
  double heading_interpolate_1st_time_last_ = 0;
  double heading_2nd_time_last_ = 0;
  double heading_interpolate_2nd_time_last_ = 0;
  double heading_3rd_time_last_ = 0;
  double heading_interpolate_3rd_time_last_ = 0;
  double yaw_rate_offset_stop_time_last_ = 0;
  double yaw_rate_offset_1st_time_last_ = 0;
  double yaw_rate_offset_2nd_time_last_ = 0;
  double slip_angle_time_last_ = 0;
  double height_time_last_ = 0;
  double pitching_time_last_ = 0;
  double enu_vel_time_last_ = 0;
  double enu_absolute_pos_time_last_ = 0;
  double enu_absolute_pos_interpolate_time_last_ = 0;
  double eagleye_twist_time_last_ = 0;

  bool use_compare_yaw_rate_ = false;
  double update_rate_ = 10.0;
  double th_gnss_deadrock_time_ = 10;
  double th_diff_rad_per_sec_ = 0.17453;
  int num_continuous_abnormal_yaw_rate_ = 0;
  int th_num_continuous_abnormal_yaw_rate_ = 10;

  std::shared_ptr<diagnostic_updater::Updater> updater_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_rtklib_fix_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr
    sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_1st_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_1st_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_3rd_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_3rd_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_stop_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_1st_;
  rclcpp::Subscription<eagleye_msgs::msg::YawrateOffset>::SharedPtr sub_yaw_rate_offset_2nd_;
  rclcpp::Subscription<eagleye_msgs::msg::SlipAngle>::SharedPtr sub_slip_angle_;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub_enu_relative_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_enu_vel_;
  rclcpp::Subscription<eagleye_msgs::msg::Height>::SharedPtr sub_height_;
  rclcpp::Subscription<eagleye_msgs::msg::Pitching>::SharedPtr sub_pitching_;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub_enu_absolute_pos_;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr
    sub_enu_absolute_pos_interpolate_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_eagleye_fix_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_eagleye_twist_;
  rclcpp::Subscription<eagleye_msgs::msg::Rolling>::SharedPtr sub_rolling_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_comparison_velocity_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_correction_velocity_;

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_ = *msg;
  }

  void rtklibFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    rtklib_fix_ = *msg;
  }

  void navsatfixGgaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
  {
    gga_ = *msg;
    gga_sub_status_ = true;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void correctionVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    correction_velocity_ = *msg;
  }

  void velocityScaleFactorCallback(
    const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
  {
    velocity_scale_factor_ = *msg;
  }

  void distanceCallback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    distance_ = *msg;
  }

  void heading1stCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_1st_ = *msg;
  }

  void headingInterpolate1stCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_1st_ = *msg;
  }

  void heading2ndCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_2nd_ = *msg;
  }

  void headingInterpolate2ndCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_2nd_ = *msg;
  }

  void heading3rdCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_3rd_ = *msg;
  }

  void headingInterpolate3rdCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_3rd_ = *msg;
  }

  void yawRateOffsetStopCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_stop_ = *msg;
  }

  void yawRateOffset1stCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_1st_ = *msg;
  }

  void yawRateOffset2ndCallback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
  {
    yaw_rate_offset_2nd_ = *msg;
  }

  void slipAngleCallback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
  {
    slip_angle_ = *msg;
  }

  void enuRelativePosCallback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    enu_relative_pos_ = *msg;
  }

  void enuVelCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    enu_vel_ = *msg;
  }

  void enuAbsolutePosCallback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    enu_absolute_pos_ = *msg;
  }

  void heightCallback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
  {
    height_ = *msg;
  }

  void pitchingCallback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
  {
    pitching_ = *msg;
  }

  void rollingCallback(const eagleye_msgs::msg::Rolling::ConstSharedPtr msg)
  {
    rolling_ = *msg;
  }

  void enuAbsolutePosInterpolateCallback(
    const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    enu_absolute_pos_interpolate_ = *msg;
  }

  void eagleyeFixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    eagleye_fix_ = *msg;
  }

  void eagleyeTwistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    eagleye_twist_ = *msg;
  }

  void comparisonVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    comparison_velocity_ptr_ = msg;
  }

  void imuTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(imu_.header.stamp);
    auto imu_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (imu_time_last_ == imu_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      msg = "not subscribed to topic";
    }

    imu_time_last_ = imu_time;
    stat.summary(level, msg);
  }

  void rtklibNavTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(rtklib_nav_.header.stamp);
    auto rtklib_nav_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (rtklib_nav_time_last_ - rtklib_nav_time > th_gnss_deadrock_time_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    }

    rtklib_nav_time_last_ = rtklib_nav_time;
    stat.summary(level, msg);
  }

  void navsatGgaTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(gga_.header.stamp);
    auto navsat_gga_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (
      navsat_gga_time_last_ - navsat_gga_time > th_gnss_deadrock_time_ || !gga_sub_status_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }

    navsat_gga_time_last_ = navsat_gga_time;
    stat.summary(level, msg);
  }

  void velocityTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(velocity_.header.stamp);
    auto velocity_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (velocity_time_last_ == velocity_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      msg = "not subscribed to topic";
    }

    velocity_time_last_ = velocity_time;
    stat.summary(level, msg);
  }

  void velocityScaleFactorTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(velocity_scale_factor_.header.stamp);
    auto velocity_scale_factor_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (velocity_scale_factor_time_last_ == velocity_scale_factor_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!velocity_scale_factor_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    } else if (velocity_scale_factor_.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (
        velocity_scale_factor_.status.error_code ==
        eagleye_msgs::msg::Status::NAN_OR_INFINITE) {
        msg = "Estimated velocity scale factor is NaN or infinete";
      } else if (
        velocity_scale_factor_.status.error_code ==
        eagleye_msgs::msg::Status::TOO_LARGE_OR_SMALL) {
        msg = "Estimated velocity scale factor is too large or too small";
      } else {
        msg = "abnormal error of velocity_scale_factor";
      }
    }

    velocity_scale_factor_time_last_ = velocity_scale_factor_time;
    stat.summary(level, msg);
  }

  void distanceTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(distance_.header.stamp);
    auto distance_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (distance_time_last_ == distance_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(distance_.distance)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (!distance_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    distance_time_last_ = distance_time;
    stat.summary(level, msg);
  }

  void heading1stTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(heading_1st_.header.stamp);
    auto heading_1st_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(heading_1st_.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (heading_1st_time_last_ - heading_1st_time > th_gnss_deadrock_time_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    } else if (!heading_1st_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    heading_1st_time_last_ = heading_1st_time;
    stat.summary(level, msg);
  }

  void headingInterpolate1stTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(heading_interpolate_1st_.header.stamp);
    auto heading_interpolate_1st_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (heading_interpolate_1st_time_last_ == heading_interpolate_1st_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(heading_interpolate_1st_.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (!heading_interpolate_1st_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    heading_interpolate_1st_time_last_ = heading_interpolate_1st_time;
    stat.summary(level, msg);
  }

  void heading2ndTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(heading_2nd_.header.stamp);
    auto heading_2nd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(heading_2nd_.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (heading_2nd_time_last_ - heading_2nd_time > th_gnss_deadrock_time_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    } else if (!heading_2nd_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    heading_2nd_time_last_ = heading_2nd_time;
    stat.summary(level, msg);
  }

  void headingInterpolate2ndTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(heading_interpolate_2nd_.header.stamp);
    auto heading_interpolate_2nd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (heading_interpolate_2nd_time_last_ == heading_interpolate_2nd_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(heading_interpolate_2nd_.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (!heading_interpolate_2nd_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    heading_interpolate_2nd_time_last_ = heading_interpolate_2nd_time;
    stat.summary(level, msg);
  }

  void heading3rdTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(heading_3rd_.header.stamp);
    auto heading_3rd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (!std::isfinite(heading_3rd_.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (heading_3rd_time_last_ - heading_3rd_time > th_gnss_deadrock_time_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    } else if (!heading_3rd_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    heading_3rd_time_last_ = heading_3rd_time;
    stat.summary(level, msg);
  }

  void headingInterpolate3rdTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(heading_interpolate_3rd_.header.stamp);
    auto heading_interpolate_3rd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (heading_interpolate_3rd_time_last_ == heading_interpolate_3rd_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(heading_interpolate_3rd_.heading_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (!heading_interpolate_3rd_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    heading_interpolate_3rd_time_last_ = heading_interpolate_3rd_time;
    stat.summary(level, msg);
  }

  void yawRateOffsetStopTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(yaw_rate_offset_stop_.header.stamp);
    auto yaw_rate_offset_stop_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (yaw_rate_offset_stop_time_last_ == yaw_rate_offset_stop_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!yaw_rate_offset_stop_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    } else if (yaw_rate_offset_stop_.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (yaw_rate_offset_stop_.status.error_code ==
          eagleye_msgs::msg::Status::NAN_OR_INFINITE) {
        msg = "estimate value is NaN or infinete";
      } else {
        msg = "abnormal error of yaw_rate_offset_stop";
      }
    }

    yaw_rate_offset_stop_time_last_ = yaw_rate_offset_stop_time;
    stat.summary(level, msg);
  }

  void yawRateOffset1stTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(yaw_rate_offset_1st_.header.stamp);
    auto yaw_rate_offset_1st_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (yaw_rate_offset_1st_time_last_ == yaw_rate_offset_1st_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!yaw_rate_offset_1st_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    } else if (yaw_rate_offset_1st_.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (yaw_rate_offset_1st_.status.error_code ==
          eagleye_msgs::msg::Status::NAN_OR_INFINITE) {
        msg = "estimate value is NaN or infinete";
      } else {
        msg = "abnormal error of yaw_rate_offset_1st";
      }
    }

    yaw_rate_offset_1st_time_last_ = yaw_rate_offset_1st_time;
    stat.summary(level, msg);
  }

  void yawRateOffset2ndTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(yaw_rate_offset_2nd_.header.stamp);
    auto yaw_rate_offset_2nd_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (yaw_rate_offset_2nd_time_last_ == yaw_rate_offset_2nd_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!yaw_rate_offset_2nd_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    } else if (yaw_rate_offset_2nd_.status.is_abnormal) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (yaw_rate_offset_2nd_.status.error_code ==
          eagleye_msgs::msg::Status::NAN_OR_INFINITE) {
        msg = "estimate value is NaN or infinete";
      } else {
        msg = "abnormal error of yaw_rate_offset_2nd";
      }
    }

    yaw_rate_offset_2nd_time_last_ = yaw_rate_offset_2nd_time;
    stat.summary(level, msg);
  }

  void slipAngleTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(slip_angle_.header.stamp);
    auto slip_angle_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (slip_angle_time_last_ == slip_angle_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(slip_angle_.slip_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (slip_angle_.coefficient == 0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "/slip_angle/manual_coefficient is not set";
    } else if (!slip_angle_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    slip_angle_time_last_ = slip_angle_time;
    stat.summary(level, msg);
  }

  void enuVelTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(enu_vel_.header.stamp);
    auto enu_vel_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (
      !std::isfinite(enu_vel_.vector.x) || !std::isfinite(enu_vel_.vector.y) ||
      !std::isfinite(enu_vel_.vector.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (enu_vel_time_last_ == enu_vel_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    }

    enu_vel_time_last_ = enu_vel_time;
    stat.summary(level, msg);
  }

  void heightTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(height_.header.stamp);
    auto height_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (height_time_last_ == height_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(height_.height)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (!height_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    height_time_last_ = height_time;
    stat.summary(level, msg);
  }

  void pitchingTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(pitching_.header.stamp);
    auto pitching_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (pitching_time_last_ == pitching_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed to topic";
    } else if (!std::isfinite(pitching_.pitching_angle)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (!pitching_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    pitching_time_last_ = pitching_time;
    stat.summary(level, msg);
  }

  void enuAbsolutePosTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(enu_absolute_pos_.header.stamp);
    auto enu_absolute_pos_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (
      !std::isfinite(enu_absolute_pos_.enu_pos.x) ||
      !std::isfinite(enu_absolute_pos_.enu_pos.y) ||
      !std::isfinite(enu_absolute_pos_.enu_pos.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (enu_absolute_pos_time_last_ - enu_absolute_pos_time > th_gnss_deadrock_time_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    } else if (!enu_absolute_pos_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    enu_absolute_pos_time_last_ = enu_absolute_pos_time;
    stat.summary(level, msg);
  }

  void enuAbsolutePosInterpolateTopicChecker(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(enu_absolute_pos_interpolate_.header.stamp);
    auto enu_absolute_pos_interpolate_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (
      !std::isfinite(enu_absolute_pos_interpolate_.enu_pos.x) ||
      !std::isfinite(enu_absolute_pos_interpolate_.enu_pos.y) ||
      !std::isfinite(enu_absolute_pos_interpolate_.enu_pos.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    } else if (
      enu_absolute_pos_interpolate_time_last_ == enu_absolute_pos_interpolate_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "not subscribed or deadlock of more than 10 seconds";
    } else if (!enu_absolute_pos_interpolate_.status.enabled_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg = "estimates have not started yet";
    }

    enu_absolute_pos_interpolate_time_last_ = enu_absolute_pos_interpolate_time;
    stat.summary(level, msg);
  }

  void twistTopicChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    rclcpp::Time ros_clock(eagleye_twist_.header.stamp);
    auto eagleye_twist_time = ros_clock.seconds();

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (eagleye_twist_time_last_ == eagleye_twist_time) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "not subscribed or deadlock of more than 10 seconds";
    } else if (
      !std::isfinite(eagleye_twist_.twist.linear.x) ||
      !std::isfinite(eagleye_twist_.twist.linear.y) ||
      !std::isfinite(eagleye_twist_.twist.linear.z) ||
      !std::isfinite(eagleye_twist_.twist.angular.x) ||
      !std::isfinite(eagleye_twist_.twist.angular.y) ||
      !std::isfinite(eagleye_twist_.twist.angular.z)) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "invalid number";
    }

    eagleye_twist_time_last_ = eagleye_twist_time;
    stat.summary(level, msg);
  }

  void imuComparisonChecker(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (comparison_velocity_ptr_ == nullptr) {
      return;
    }

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg = "OK";

    if (
      use_compare_yaw_rate_ &&
      th_diff_rad_per_sec_ <
        std::abs(
          corrected_imu_.angular_velocity.z -
          comparison_velocity_ptr_->twist.angular.z)) {
      num_continuous_abnormal_yaw_rate_++;
    } else {
      num_continuous_abnormal_yaw_rate_ = 0;
    }

    if (num_continuous_abnormal_yaw_rate_ > th_num_continuous_abnormal_yaw_rate_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg = "IMU Yaw Rate too large or too small compared to reference twist";
    }
    stat.summary(level, msg);
  }

  void printStatus()
  {
    std::cout << std::endl;
    std::cout << "\033[1;33m Eagleye status \033[m" << std::endl;
    std::cout << std::endl;
    std::cout << std::fixed;

    std::cout << "--- \033[1;34m imu(input)\033[m ------------------------------" << std::endl;
    std::cout << "\033[1m linear_acceleration \033[mx " << std::setprecision(6)
              << imu_.linear_acceleration.x << " [m/s^2]" << std::endl;
    std::cout << "\033[1m linear acceleration \033[my " << std::setprecision(6)
              << imu_.linear_acceleration.y << " [m/s^2]" << std::endl;
    std::cout << "\033[1m linear acceleration \033[mz " << std::setprecision(6)
              << imu_.linear_acceleration.z << " [m/s^2]" << std::endl;
    std::cout << "\033[1m angular velocity \033[mx " << std::setprecision(6)
              << imu_.angular_velocity.x << " [rad/s]" << std::endl;
    std::cout << "\033[1m angular velocity \033[my " << std::setprecision(6)
              << imu_.angular_velocity.y << " [rad/s]" << std::endl;
    std::cout << "\033[1m angular velocity \033[mz " << std::setprecision(6)
              << imu_.angular_velocity.z << " [rad/s]" << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m velocity(input)\033[m -------------------------" << std::endl;
    std::cout << "\033[1m velocity \033[m" << std::setprecision(4)
              << velocity_.twist.linear.x * 3.6 << " [km/h]" << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m rtklib(input)\033[m ---------------------------" << std::endl;
    std::cout << "\033[1m time of week  \033[m" << rtklib_nav_.tow << " [ms]" << std::endl;
    std::cout << "\033[1m latitude  \033[m" << std::setprecision(8)
              << rtklib_nav_.status.latitude << " [deg]" << std::endl;
    std::cout << "\033[1m longitude  \033[m" << std::setprecision(8)
              << rtklib_nav_.status.longitude << " [deg]" << std::endl;
    std::cout << "\033[1m altitude  \033[m" << std::setprecision(4)
              << rtklib_nav_.status.altitude << " [m]" << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m gga(input)\033[m ------------------------------" << std::endl;

    if (gga_sub_status_) {
      std::cout << "\033[1m rtk status \033[m " << int(gga_.gps_qual) << std::endl;
      std::cout << "\033[1m rtk status \033[m "
                << (int(gga_.gps_qual) != 4 ? "\033[1;31mNo Fix\033[m" : "\033[1;32mFix\033[m")
                << std::endl;
      std::cout << "\033[1m latitude  \033[m" << std::setprecision(8) << gga_.lat << " [deg]"
                << std::endl;
      std::cout << "\033[1m longitude  \033[m" << std::setprecision(8) << gga_.lon << " [deg]"
                << std::endl;
      std::cout << "\033[1m altitude  \033[m" << std::setprecision(4)
                << gga_.alt + gga_.undulation << " [m]" << std::endl;
      std::cout << std::endl;
    } else {
      std::cout << std::endl;
      std::cout << "\033[1;31m no subscription \033[m" << std::endl;
      std::cout << std::endl;
    }

    std::cout << "--- \033[1;34m velocity SF\033[m -----------------------------" << std::endl;
    std::cout << "\033[1m scale factor \033[m " << std::setprecision(4)
              << velocity_scale_factor_.scale_factor << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (velocity_scale_factor_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                               : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m yaw_rate offset stop\033[m ---------------------" << std::endl;
    std::cout << "\033[1m yaw_rate offset \033[m " << std::setprecision(6)
              << yaw_rate_offset_stop_.yaw_rate_offset << " [rad/s]" << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (yaw_rate_offset_stop_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                              : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m yaw_rate offset\033[m --------------------------" << std::endl;
    std::cout << "\033[1m yaw_rate offset \033[m " << std::setprecision(6)
              << yaw_rate_offset_2nd_.yaw_rate_offset << " [rad/s]" << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (yaw_rate_offset_2nd_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                             : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m slip angle\033[m ------------------------------" << std::endl;
    std::cout << "\033[1m coefficient \033[m " << std::setprecision(6)
              << slip_angle_.coefficient << std::endl;
    std::cout << "\033[1m slip angle \033[m " << std::setprecision(6)
              << slip_angle_.slip_angle << " [rad]" << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (slip_angle_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                    : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m heading\033[m ---------------------------------" << std::endl;
    std::cout << "\033[1m heading \033[m " << std::setprecision(6)
              << heading_interpolate_3rd_.heading_angle << " [rad/s]" << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (heading_interpolate_3rd_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                                  : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m pitching\033[m --------------------------------" << std::endl;
    std::cout << "\033[1m pitching \033[m " << std::setprecision(6)
              << pitching_.pitching_angle << " [rad]" << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (pitching_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                  : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m height\033[m ----------------------------------" << std::endl;
    std::cout << "\033[1m height \033[m " << std::setprecision(4) << height_.height << " [m]"
              << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (height_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;

    std::cout << "--- \033[1;34m position\033[m --------------------------------" << std::endl;
    std::cout << "\033[1m latitude  \033[m" << std::setprecision(8) << eagleye_fix_.latitude
              << " [deg]" << std::endl;
    std::cout << "\033[1m longitude  \033[m" << std::setprecision(8) << eagleye_fix_.longitude
              << " [deg]" << std::endl;
    std::cout << "\033[1m altitude  \033[m" << std::setprecision(4) << eagleye_fix_.altitude
              << " [m]" << std::endl;
    std::cout << "\033[1m status enable \033[m "
              << (enu_absolute_pos_interpolate_.status.enabled_status ? "\033[1;32mTrue\033[m"
                                                                      : "\033[1;31mFalse\033[m")
              << std::endl;
    std::cout << std::endl;
  }

  void outputLog()
  {
    if (!log_header_make_) {
      std::ofstream output_log_file(
        output_log_dir_, std::ios_base::trunc | std::ios_base::out);
      std::cout << "Output file = eagleye_log.csv" << std::endl;
      output_log_file
        << "timestamp,imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,"
           "imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z"
        << ",rtklib_nav.tow,rtklib_nav.ecef_pos.x,rtklib_nav.ecef_pos.y,rtklib_nav.ecef_pos.z,"
           "rtklib_nav.ecef_vel.x,rtklib_nav.ecef_vel.y,rtklib_nav.ecef_vel.z,rtklib_nav.status."
           "status.status,rtklib_nav.status.status.service,rtklib_nav.status.latitude,rtklib_nav."
           "status.longitude,rtklib_nav.status.altitude"
        << ",velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z,velocity."
           "twist.angular.x,velocity.twist.angular.y,velocity.twist.angular.z"
        << ",velocity_scale_factor.scale_factor,correction_velocity.twist.linear.x,correction_"
           "velocity.twist.linear.y,correction_velocity.twist.linear.z,correction_velocity.twist."
           "angular.x,correction_velocity.twist.angular.y,correction_velocity.twist.angular.z,"
           "velocity_scale_factor.status.enabled_status,velocity_scale_factor.status.estimate_"
           "status"
        << ",distance.distance,distance.status.enabled_status,distance.status.estimate_status"
        << ",heading_1st.heading_angle,heading_1st.status.enabled_status,heading_1st.status."
           "estimate_status"
        << ",heading_interpolate_1st.heading_angle,heading_interpolate_1st.status.enabled_status,"
           "heading_interpolate_1st.status.estimate_status"
        << ",heading_2nd.heading_angle,heading_2nd.status.enabled_status,heading_2nd.status."
           "estimate_status"
        << ",heading_interpolate_2nd.heading_angle,heading_interpolate_2nd.status.enabled_status,"
           "heading_interpolate_2nd.status.estimate_status"
        << ",heading_3rd.heading_angle,heading_3rd.status.enabled_status,heading_3rd.status."
           "estimate_status"
        << ",heading_interpolate_3rd.heading_angle,heading_interpolate_3rd.status.enabled_status,"
           "heading_interpolate_3rd.status.estimate_status"
        << ",yaw_rate_offset_stop.yaw_rate_offset,yaw_rate_offset_stop.status.enabled_status,"
           "yaw_rate_offset_stop.status.estimate_status"
        << ",yaw_rate_offset_1st.yaw_rate_offset,yaw_rate_offset_1st.status.enabled_status,"
           "yaw_rate_offset_1st.status.estimate_status"
        << ",yaw_rate_offset_2nd.yaw_rate_offset,yaw_rate_offset_2nd.status.enabled_status,"
           "yaw_rate_offset_2nd.status.estimate_status"
        << ",slip_angle.coefficient,slip_angle.slip_angle,slip_angle.status.enabled_status,"
           "slip_angle.status.estimate_status"
        << ",enu_vel.vector.x,enu_vel.vector.y,enu_vel.vector.z"
        << ",enu_absolute_pos.enu_pos.x,enu_absolute_pos.enu_pos.y,enu_absolute_pos.enu_pos.z,"
           "enu_absolute_pos.ecef_base_pos.x,enu_absolute_pos.ecef_base_pos.y,enu_absolute_pos."
           "ecef_base_pos.z,enu_absolute_pos.status.enabled_status,enu_absolute_pos.status."
           "estimate_status"
        << ",enu_absolute_pos_interpolate.enu_pos.x,enu_absolute_pos_interpolate.enu_pos.y,"
           "enu_absolute_pos_interpolate.enu_pos.z,enu_absolute_pos_interpolate.ecef_base_pos.x,"
           "enu_absolute_pos_interpolate.ecef_base_pos.y,enu_absolute_pos_interpolate.ecef_base_"
           "pos.z,enu_absolute_pos_interpolate.status.enabled_status,enu_absolute_pos_interpolate."
           "status.estimate_status"
        << ",height.height,height.status.enabled_status,height.status.estimate_status"
        << ",pitching.pitching_angle,pitching.status.enabled_status,pitching.status.estimate_status"
        << ",acc_x_offset.acc_x_offset,acc_x_offset.status.enabled_status,acc_x_offset.status."
           "estimate_status"
        << ",acc_x_scale_factor.acc_x_scale_factor,acc_x_scale_factor.status.enabled_status,"
           "acc_x_scale_factor.status.estimate_status"
        << ",rolling.rolling_angle,rolling.status.enabled_status,rolling.status.estimate_status"
        << ",gga_timestamp"
        << ",gga_llh.latitude,gga_llh.longitude,gga_llh.altitude"
        << ",gga_llh.gps_qual"
        << ",eagleye_pp_llh.latitude,eagleye_pp_llh.longitude,eagleye_pp_llh.altitude"
        << ",eagleye_pp_llh.orientation_covariance[0],eagleye_pp_llh.orientation_covariance[1],"
           "eagleye_pp_llh.orientation_covariance[2],eagleye_pp_llh.orientation_covariance[3],"
           "eagleye_pp_llh.orientation_covariance[4],eagleye_pp_llh.orientation_covariance[5],"
           "eagleye_pp_llh.orientation_covariance[6],eagleye_pp_llh.orientation_covariance[7],"
           "eagleye_pp_llh.orientation_covariance[8]"
        << ",eagleye_pp_llh.status"
        << ",eagleye_pp_llh.height_status"
        << ",enu_relative_pos.enu_pos.x,enu_relative_pos.enu_pos.y,enu_relative_pos.enu_pos.z"
        << ",enu_relative_pos.status.enabled_status"
        << std::endl;
      log_header_make_ = true;
    } else {
      std::ofstream output_log_file(output_log_dir_, std::ios_base::app);
      rclcpp::Time imu_clock(imu_.header.stamp);
      long double nano_sec = imu_clock.nanoseconds();
      long double sec_digits = std::pow(10, 9);
      long double imu_time = nano_sec / sec_digits;
      output_log_file << std::fixed << std::setprecision(9) << imu_time << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << imu_.angular_velocity.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << imu_.angular_velocity.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << imu_.angular_velocity.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << imu_.linear_acceleration.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << imu_.linear_acceleration.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << imu_.linear_acceleration.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10)
                      << rtklib_nav_.tow << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.ecef_pos.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.ecef_pos.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.ecef_pos.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.ecef_vel.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.ecef_vel.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.ecef_vel.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10)
                      << int(rtklib_nav_.status.status.status) << ",";
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10)
                      << rtklib_nav_.status.status.service << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.status.latitude << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.status.longitude << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rtklib_nav_.status.altitude << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_.twist.linear.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_.twist.linear.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_.twist.linear.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_.twist.angular.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_.twist.angular.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_.twist.angular.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << velocity_scale_factor_.scale_factor << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << correction_velocity_.twist.linear.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << correction_velocity_.twist.linear.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << correction_velocity_.twist.linear.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << correction_velocity_.twist.angular.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << correction_velocity_.twist.angular.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << correction_velocity_.twist.angular.z << ",";
      output_log_file << (velocity_scale_factor_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (velocity_scale_factor_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << distance_.distance << ",";
      output_log_file << (distance_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (distance_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << heading_1st_.heading_angle << ",";
      output_log_file << (heading_1st_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (heading_1st_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << heading_interpolate_1st_.heading_angle << ",";
      output_log_file << (heading_interpolate_1st_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (heading_interpolate_1st_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << heading_2nd_.heading_angle << ",";
      output_log_file << (heading_2nd_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (heading_2nd_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << heading_interpolate_2nd_.heading_angle << ",";
      output_log_file << (heading_interpolate_2nd_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (heading_interpolate_2nd_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << heading_3rd_.heading_angle << ",";
      output_log_file << (heading_3rd_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (heading_3rd_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << heading_interpolate_3rd_.heading_angle << ",";
      output_log_file << (heading_interpolate_3rd_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (heading_interpolate_3rd_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << yaw_rate_offset_stop_.yaw_rate_offset << ",";
      output_log_file << (yaw_rate_offset_stop_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (yaw_rate_offset_stop_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << yaw_rate_offset_1st_.yaw_rate_offset << ",";
      output_log_file << (yaw_rate_offset_1st_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (yaw_rate_offset_1st_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << yaw_rate_offset_2nd_.yaw_rate_offset << ",";
      output_log_file << (yaw_rate_offset_2nd_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (yaw_rate_offset_2nd_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << slip_angle_.coefficient << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << slip_angle_.slip_angle << ",";
      output_log_file << (slip_angle_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (slip_angle_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_vel_.vector.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_vel_.vector.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_vel_.vector.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_.enu_pos.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_.enu_pos.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_.enu_pos.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_.ecef_base_pos.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_.ecef_base_pos.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_.ecef_base_pos.z << ",";
      output_log_file << (enu_absolute_pos_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (enu_absolute_pos_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_interpolate_.enu_pos.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_interpolate_.enu_pos.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_interpolate_.enu_pos.z << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_interpolate_.ecef_base_pos.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_interpolate_.ecef_base_pos.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_absolute_pos_interpolate_.ecef_base_pos.z << ",";
      output_log_file << (enu_absolute_pos_interpolate_.status.enabled_status ? "1" : "0")
                      << ",";
      output_log_file << (enu_absolute_pos_interpolate_.status.estimate_status ? "1" : "0")
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << height_.height << ",";
      output_log_file << (height_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (height_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << pitching_.pitching_angle << ",";
      output_log_file << (pitching_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (pitching_.status.estimate_status ? "1" : "0") << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";  // acc_x_offset
      output_log_file << 0 << ",";  // acc_x_offset.status.enabled_status
      output_log_file << 0 << ",";  // acc_x_offset.status.estimate_status
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";  // acc_x_scale_factor.acc_x_scale_factor
      output_log_file << 0 << ",";  // acc_x_scale_factor.status.enabled_status
      output_log_file << 0 << ",";  // acc_x_scale_factor.status.estimate_status
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << rolling_.rolling_angle << ",";
      output_log_file << (rolling_.status.enabled_status ? "1" : "0") << ",";
      output_log_file << (rolling_.status.estimate_status ? "1" : "0") << ",";
      rclcpp::Time gga_clock(gga_.header.stamp);
      double gga_time = gga_clock.seconds();
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_time
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_.lat
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_.lon
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << gga_.alt + gga_.undulation << ",";
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10)
                      << int(gga_.gps_qual) << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << eagleye_fix_.latitude << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << eagleye_fix_.longitude << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << eagleye_fix_.altitude << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0
                      << ",";
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ",";
      output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_relative_pos_.enu_pos.x << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_relative_pos_.enu_pos.y << ",";
      output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10)
                      << enu_relative_pos_.enu_pos.z << ",";
      output_log_file << (enu_relative_pos_.status.enabled_status ? "1" : "0");
      output_log_file << "\n";
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    imu_.header = msg->header;
    imu_.orientation = msg->orientation;
    imu_.orientation_covariance = msg->orientation_covariance;
    imu_.angular_velocity = msg->angular_velocity;
    imu_.angular_velocity_covariance = msg->angular_velocity_covariance;
    imu_.linear_acceleration = msg->linear_acceleration;
    imu_.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    if (print_status_) {
      printStatus();
    }

    if (log_output_status_) {
      outputLog();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  return 0;
}
