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
 * position_interpolate.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"
#include "rclcpp/rclcpp.hpp"

class PositionInterpolateNode : public rclcpp::Node
{
public:
  PositionInterpolateNode() : Node("eagleye_position_interpolate")
  {
    std::string subscribe_gga_topic_name = "gnss/gga";

    std::string yaml_file;
    this->declare_parameter("yaml_file", yaml_file);
    this->get_parameter("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    try {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      position_interpolate_parameter_.imu_rate =
        conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
      position_interpolate_parameter_.stop_judgment_threshold =
        conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
      position_interpolate_parameter_.sync_search_period =
        conf["/**"]["ros__parameters"]["position_interpolate"]["sync_search_period"].as<double>();
      position_interpolate_parameter_.proc_noise =
        conf["/**"]["ros__parameters"]["position_interpolate"]["proc_noise"].as<double>();

      std::cout << "imu_rate " << position_interpolate_parameter_.imu_rate << std::endl;
      std::cout << "stop_judgment_threshold "
                << position_interpolate_parameter_.stop_judgment_threshold << std::endl;
      std::cout << "sync_search_period " << position_interpolate_parameter_.sync_search_period
                << std::endl;
    } catch (YAML::Exception& e) {
      std::cerr << "\033[1;31mheading_interpolate Node YAML Error: " << e.msg << "\033[0m"
                << std::endl;
      exit(3);
    }

    sub_enu_vel_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "enu_vel", rclcpp::QoS(10),
      std::bind(&PositionInterpolateNode::enuVelCallback, this, std::placeholders::_1));
    sub_enu_absolute_pos_ = this->create_subscription<eagleye_msgs::msg::Position>(
      "enu_absolute_pos", rclcpp::QoS(10),
      std::bind(&PositionInterpolateNode::enuAbsolutePosCallback, this, std::placeholders::_1));
    sub_gnss_smooth_pos_ = this->create_subscription<eagleye_msgs::msg::Position>(
      "gnss_smooth_pos_enu", rclcpp::QoS(10),
      std::bind(
        &PositionInterpolateNode::gnssSmootPosEnuCallback, this, std::placeholders::_1));
    sub_height_ = this->create_subscription<eagleye_msgs::msg::Height>(
      "height", rclcpp::QoS(10),
      std::bind(&PositionInterpolateNode::heightCallback, this, std::placeholders::_1));
    sub_gga_ = this->create_subscription<nmea_msgs::msg::Gpgga>(
      subscribe_gga_topic_name, rclcpp::QoS(10),
      std::bind(&PositionInterpolateNode::ggaCallback, this, std::placeholders::_1));
    sub_heading_interpolate_3rd_ = this->create_subscription<eagleye_msgs::msg::Heading>(
      "heading_interpolate_3rd", rclcpp::QoS(10),
      std::bind(
        &PositionInterpolateNode::headingInterpolate3rdCallback, this, std::placeholders::_1));
    pub_pos_ = this->create_publisher<eagleye_msgs::msg::Position>(
      "enu_absolute_pos_interpolate", rclcpp::QoS(10));
    pub_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10));
  }

private:
  eagleye_msgs::msg::Position enu_absolute_pos_;
  geometry_msgs::msg::Vector3Stamped enu_vel_;
  eagleye_msgs::msg::Height height_;
  eagleye_msgs::msg::Position gnss_smooth_pos_;
  nmea_msgs::msg::Gpgga gga_;
  eagleye_msgs::msg::Heading heading_interpolate_3rd_;
  eagleye_msgs::msg::Position enu_absolute_pos_interpolate_;
  sensor_msgs::msg::NavSatFix eagleye_fix_;
  PositionInterpolateParameter position_interpolate_parameter_;
  PositionInterpolateStatus position_interpolate_status_ = {};

  rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub_pos_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_enu_vel_;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub_enu_absolute_pos_;
  rclcpp::Subscription<eagleye_msgs::msg::Position>::SharedPtr sub_gnss_smooth_pos_;
  rclcpp::Subscription<eagleye_msgs::msg::Height>::SharedPtr sub_height_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr sub_gga_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_interpolate_3rd_;

  void ggaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg) { gga_ = *msg; }

  void enuAbsolutePosCallback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    enu_absolute_pos_ = *msg;
  }

  void gnssSmootPosEnuCallback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
  {
    gnss_smooth_pos_ = *msg;
  }

  void heightCallback(const eagleye_msgs::msg::Height::ConstSharedPtr msg) { height_ = *msg; }

  void headingInterpolate3rdCallback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
  {
    heading_interpolate_3rd_ = *msg;
  }

  void enuVelCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    rclcpp::Time ros_clock(gga_.header.stamp);
    auto gga_time = ros_clock.seconds();

    enu_vel_ = *msg;
    enu_absolute_pos_interpolate_.header = msg->header;
    enu_absolute_pos_interpolate_.header.frame_id = "base_link";
    eagleye_fix_.header = msg->header;
    eagleye_fix_.header.frame_id = "gnss";
    position_interpolate_estimate(
      enu_absolute_pos_, enu_vel_, gnss_smooth_pos_, height_, heading_interpolate_3rd_,
      position_interpolate_parameter_, &position_interpolate_status_,
      &enu_absolute_pos_interpolate_, &eagleye_fix_);
    if (enu_absolute_pos_.status.enabled_status == true) {
      if (eagleye_fix_.latitude == 0 && eagleye_fix_.longitude == 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "eagleye_fix is not published because latitude and longitude are 0.");
      } else {
        pub_pos_->publish(enu_absolute_pos_interpolate_);
        pub_fix_->publish(eagleye_fix_);
      }
    } else if (gga_time != 0) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header = gga_.header;
      fix.latitude = gga_.lat;
      fix.longitude = gga_.lon;
      fix.altitude = gga_.alt + gga_.undulation;
      pub_fix_->publish(fix);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionInterpolateNode>());
  return 0;
}
