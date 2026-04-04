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
 * position.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

class PositionNode : public rclcpp::Node
{
public:
  PositionNode()
  : Node("eagleye_position"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
    std::string subscribe_gga_topic_name = "gnss/gga";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_gnss_mode_ = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();
      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();

      position_parameter_.ecef_base_pos_x =
        conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
      position_parameter_.ecef_base_pos_y =
        conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
      position_parameter_.ecef_base_pos_z =
        conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();
      position_parameter_.tf_gnss_parent_frame =
        conf["/**"]["ros__parameters"]["tf_gnss_frame"]["parent"].as<std::string>();
      position_parameter_.tf_gnss_child_frame =
        conf["/**"]["ros__parameters"]["tf_gnss_frame"]["child"].as<std::string>();
      position_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      position_parameter_.gnss_rate =
        conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      position_parameter_.moving_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
      position_parameter_.estimated_interval =
        conf["/**"]["ros__parameters"]["position"]["estimated_interval"].as<double>();
      position_parameter_.update_distance =
        conf["/**"]["ros__parameters"]["position"]["update_distance"].as<double>();
      position_parameter_.outlier_threshold =
        conf["/**"]["ros__parameters"]["position"]["outlier_threshold"].as<double>();
      position_parameter_.gnss_receiving_threshold =
        conf["/**"]["ros__parameters"]["position"]["gnss_receiving_threshold"].as<double>();
      position_parameter_.outlier_ratio_threshold =
        conf["/**"]["ros__parameters"]["position"]["outlier_ratio_threshold"].as<double>();
      position_parameter_.gnss_error_covariance =
        conf["/**"]["ros__parameters"]["position"]["gnss_error_covariance"].as<double>();

      std::cout << "use_gnss_mode " << use_gnss_mode_ << std::endl;
      std::cout << "use_can_less_mode " << use_can_less_mode_ << std::endl;
      std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name
                << std::endl;
      std::cout << "ecef_base_pos_x " << position_parameter_.ecef_base_pos_x << std::endl;
      std::cout << "ecef_base_pos_y " << position_parameter_.ecef_base_pos_y << std::endl;
      std::cout << "ecef_base_pos_z " << position_parameter_.ecef_base_pos_z << std::endl;
      std::cout << "tf_gnss_frame/parent " << position_parameter_.tf_gnss_parent_frame
                << std::endl;
      std::cout << "tf_gnss_frame/child " << position_parameter_.tf_gnss_child_frame << std::endl;
      std::cout << "imu_rate " << position_parameter_.imu_rate << std::endl;
      std::cout << "gnss_rate " << position_parameter_.gnss_rate << std::endl;
      std::cout << "moving_judgment_threshold " << position_parameter_.moving_judgment_threshold
                << std::endl;
      std::cout << "estimated_interval " << position_parameter_.estimated_interval << std::endl;
      std::cout << "update_distance " << position_parameter_.update_distance << std::endl;
      std::cout << "outlier_threshold " << position_parameter_.outlier_threshold << std::endl;
      std::cout << "gnss_receiving_threshold " << position_parameter_.gnss_receiving_threshold
                << std::endl;
      std::cout << "outlier_ratio_threshold " << position_parameter_.outlier_ratio_threshold
                << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mposition Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub_enu_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "enu_vel", 1000,
      std::bind(&PositionNode::enuVelCallback, this, std::placeholders::_1));
    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      subscribe_rtklib_nav_topic_name, 1000,
      std::bind(&PositionNode::rtklibNavCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&PositionNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&PositionNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_velocity_scale_factor_ = this->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>(
      "velocity_scale_factor", 1000,
      std::bind(&PositionNode::velocityScaleFactorCallback, this, std::placeholders::_1));
    sub_distance_ = this->create_subscription<eagleye_msgs::msg::Distance>(
      "distance", 1000,
      std::bind(&PositionNode::distanceCallback, this, std::placeholders::_1));
    sub_heading_interpolate_3rd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_3rd", 1000,
      std::bind(&PositionNode::headingInterpolate3rdCallback, this, std::placeholders::_1));
    sub_gga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      subscribe_gga_topic_name, 1000,
      std::bind(&PositionNode::ggaCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<eagleye_msgs::msg::Position>("enu_absolute_pos", 1000);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PositionNode::onTimer, this));
  }

private:
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor_;
  eagleye_msgs::msg::Distance distance_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  eagleye_msgs::msg::Position enu_absolute_pos_;
  geometry_msgs::msg::Vector3Stamped enu_vel_;
  nmea_msgs::msg::Gpgga gga_;
  PositionParameter position_parameter_;
  PositionStatus position_status_;
  std::string use_gnss_mode_;
  bool use_can_less_mode_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_enu_vel_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr
    sub_velocity_scale_factor_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr sub_distance_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_3rd_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    rtklib_nav_ = *msg;
  }

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
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

  void distanceCallback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    distance_ = *msg;
  }

  void headingInterpolate3rdCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_3rd_ = *msg;
  }

  void ggaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg) { gga_ = *msg; }

  void onTimer()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform(
        position_parameter_.tf_gnss_parent_frame, position_parameter_.tf_gnss_child_frame,
        tf2::TimePointZero);

      position_parameter_.tf_gnss_translation_x = transformStamped.transform.translation.x;
      position_parameter_.tf_gnss_translation_y = transformStamped.transform.translation.y;
      position_parameter_.tf_gnss_translation_z = transformStamped.transform.translation.z;
      position_parameter_.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
      position_parameter_.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
      position_parameter_.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
      position_parameter_.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  void enuVelCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    eagleye_msgs::msg::StatusStamped velocity_enable_status;
    if (use_can_less_mode_) {
      velocity_enable_status = velocity_status_;
    } else {
      velocity_enable_status.header = velocity_scale_factor_.header;
      velocity_enable_status.status = velocity_scale_factor_.status;
    }

    enu_vel_ = *msg;
    enu_absolute_pos_.header = msg->header;
    enu_absolute_pos_.header.frame_id = "base_link";
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      position_estimate(
        rtklib_nav_, velocity_, velocity_enable_status, distance_, heading_interpolate_3rd_,
        enu_vel_, position_parameter_, &position_status_, &enu_absolute_pos_);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      position_estimate(
        gga_, velocity_, velocity_enable_status, distance_, heading_interpolate_3rd_, enu_vel_,
        position_parameter_, &position_status_, &enu_absolute_pos_);
    if (enu_absolute_pos_.status.estimate_status == true) {
      pub_->publish(enu_absolute_pos_);
    }
    enu_absolute_pos_.status.estimate_status = false;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionNode>());
  return 0;
}
