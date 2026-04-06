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
 * smoothing.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class SmoothingNode : public rclcpp::Node
{
public:
  SmoothingNode()
  : Node("eagleye_smoothing"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      use_can_less_mode_ = conf["/**"]["ros__parameters"]["use_can_less_mode"].as<bool>();
      position_parameter_.tf_gnss_parent_frame =
        conf["/**"]["ros__parameters"]["tf_gnss_frame"]["parent"].as<std::string>();
      position_parameter_.tf_gnss_child_frame =
        conf["/**"]["ros__parameters"]["tf_gnss_frame"]["child"].as<std::string>();
      smoothing_parameter_.ecef_base_pos_x =
        conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
      smoothing_parameter_.ecef_base_pos_y =
        conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
      smoothing_parameter_.ecef_base_pos_z =
        conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();
      smoothing_parameter_.gnss_rate =
        conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
      smoothing_parameter_.moving_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
      smoothing_parameter_.moving_average_time =
        conf["/**"]["ros__parameters"]["smoothing"]["moving_average_time"].as<double>();
      smoothing_parameter_.moving_ratio_threshold =
        conf["/**"]["ros__parameters"]["smoothing"]["moving_ratio_threshold"].as<double>();

      std::cout << "use_can_less_mode " << use_can_less_mode_ << std::endl;
      std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name
                << std::endl;
      std::cout << "ecef_base_pos_x " << smoothing_parameter_.ecef_base_pos_x << std::endl;
      std::cout << "ecef_base_pos_y " << smoothing_parameter_.ecef_base_pos_y << std::endl;
      std::cout << "ecef_base_pos_z " << smoothing_parameter_.ecef_base_pos_z << std::endl;
      std::cout << "gnss_rate " << smoothing_parameter_.gnss_rate << std::endl;
      std::cout << "moving_judgment_threshold " << smoothing_parameter_.moving_judgment_threshold
                << std::endl;
      std::cout << "moving_average_time " << smoothing_parameter_.moving_average_time << std::endl;
      std::cout << "moving_ratio_threshold " << smoothing_parameter_.moving_ratio_threshold
                << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31msmoothing Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity", rclcpp::QoS(10),
      std::bind(&SmoothingNode::velocityCallback, this, std::placeholders::_1));
    sub_velocity_status_ = this->create_subscription<eagleye_msgs::msg::StatusStamped>(
      "velocity_status", rclcpp::QoS(10),
      std::bind(&SmoothingNode::velocityStatusCallback, this, std::placeholders::_1));
    sub_rtklib_nav_ = this->create_subscription<rtklib_msgs::msg::RtklibNav>(
      subscribe_rtklib_nav_topic_name, 1000,
      std::bind(&SmoothingNode::rtklibNavCallback, this, std::placeholders::_1));
    pub_ =
      this->create_publisher<eagleye_msgs::msg::Position>("gnss_smooth_pos_enu", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&SmoothingNode::onTimer, this));
  }

private:
  rtklib_msgs::msg::RtklibNav rtklib_nav_;
  eagleye_msgs::msg::Position gnss_smooth_pos_enu_;
  geometry_msgs::msg::TwistStamped velocity_;
  eagleye_msgs::msg::StatusStamped velocity_status_;
  PositionParameter position_parameter_;
  SmoothingParameter smoothing_parameter_;
  SmoothingStatus smoothing_status_ = {};
  bool use_can_less_mode_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_velocity_;
  rclcpp::Subscription<eagleye_msgs::msg::StatusStamped>::SharedPtr sub_velocity_status_;
  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr sub_rtklib_nav_;

  void velocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    velocity_ = *msg;
  }

  void velocityStatusCallback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
  {
    velocity_status_ = *msg;
  }

  void rtklibNavCallback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
  {
    if (use_can_less_mode_ && !velocity_status_.status.enabled_status) return;

    rtklib_nav_ = *msg;
    gnss_smooth_pos_enu_.header = msg->header;
    gnss_smooth_pos_enu_.header.frame_id = "base_link";
    smoothing_estimate(
      rtklib_nav_, velocity_, smoothing_parameter_, &smoothing_status_, &gnss_smooth_pos_enu_);
    gnss_smooth_pos_enu_.enu_pos.z -= position_parameter_.tf_gnss_translation_z;
    pub_->publish(gnss_smooth_pos_enu_);
  }

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
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmoothingNode>());
  return 0;
}
