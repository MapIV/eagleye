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
 * geo_pose_fusion.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include <geographic_msgs/msg/geo_pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <eagleye_msgs/msg/rolling.hpp>
#include <eagleye_msgs/msg/pitching.hpp>
#include <eagleye_msgs/msg/heading.hpp>
#include <eagleye_msgs/msg/position.hpp>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <ament_index_cpp/get_package_share_directory.hpp>

rclcpp::Clock _ros_clock(RCL_ROS_TIME);

static eagleye_msgs::msg::Rolling _eagleye_rolling;
static eagleye_msgs::msg::Pitching _eagleye_pitching;
static eagleye_msgs::msg::Heading _eagleye_heading;
static geometry_msgs::msg::Quaternion _quat;

rclcpp::Publisher<geographic_msgs::msg::GeoPoseWithCovarianceStamped>::SharedPtr _pub;
static geographic_msgs::msg::GeoPoseWithCovarianceStamped _geo_pose_with_covariance;

bool _fix_only_publish = false;
int _fix_judgement_type = 0;
double _fix_std_pos_thres = 0.1; // [m]

std::string _node_name = "eagleye_geo_pose_fusion";

void heading_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  _eagleye_heading = *msg;
}

void rolling_callback(const eagleye_msgs::msg::Rolling::ConstSharedPtr msg)
{
  _eagleye_rolling = *msg;
}

void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
{
  _eagleye_pitching = *msg;
}

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  bool fix_flag = false;
  if(_fix_judgement_type == 0)
  {
    if(msg->status.status == 0 && _eagleye_heading.status.enabled_status) fix_flag = true;
  }
  else if(_fix_judgement_type == 1)
  {
    if(msg->position_covariance[0] < _fix_std_pos_thres * _fix_std_pos_thres && _eagleye_heading.status.enabled_status) fix_flag = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(_node_name), "fix_judgement_type is not valid");
    rclcpp::shutdown();
  }

  if(_fix_only_publish && !fix_flag)
  {
    return;
  }

  double eagleye_heading = 0;
  tf2::Quaternion localization_quat;
  if (_eagleye_heading.status.enabled_status)
  {
    // NOTE: currently geo_pose_fusion ignores roll and pitch for robust estimation results.
    eagleye_heading = fmod((90* M_PI / 180)-_eagleye_heading.heading_angle,2*M_PI);
    localization_quat.setRPY(0, 0, eagleye_heading);
  }
  else
  {
    tf2::Matrix3x3(localization_quat).setRPY(0, 0, 0);
  }
  _quat = tf2::toMsg(localization_quat);


  _geo_pose_with_covariance.header = msg->header;
  _geo_pose_with_covariance.header.frame_id = "map";
  _geo_pose_with_covariance.pose.pose.position.latitude = msg->latitude;
  _geo_pose_with_covariance.pose.pose.position.longitude = msg->longitude;
  _geo_pose_with_covariance.pose.pose.position.altitude = msg->altitude;
  _geo_pose_with_covariance.pose.pose.orientation = _quat;

  // TODO(Map IV): temporary value
  double std_dev_roll = 100; // [rad]
  double std_dev_pitch = 100; // [rad]
  double std_dev_yaw = 100; // [rad]
  if(_eagleye_rolling.status.enabled_status) std_dev_roll = 0.5 / 180 * M_PI;
  if(_eagleye_pitching.status.enabled_status) std_dev_pitch = 0.5 / 180 * M_PI;
  if(_eagleye_heading.status.enabled_status) std_dev_yaw = std::sqrt(_eagleye_heading.variance);

  // Covariance in NavSatFix is in ENU coordinate while the one in GeoPoseWithCovariance is in Lat/Lon/Alt coordinate.
  // In order to be consistent with the msg definition, we need to swap the covariance of x and y.
  _geo_pose_with_covariance.pose.covariance[0] = msg->position_covariance[4];
  _geo_pose_with_covariance.pose.covariance[7] = msg->position_covariance[0];
  _geo_pose_with_covariance.pose.covariance[14] = msg->position_covariance[8];

  _geo_pose_with_covariance.pose.covariance[21] = std_dev_roll * std_dev_roll;
  _geo_pose_with_covariance.pose.covariance[28] = std_dev_pitch * std_dev_pitch;
  _geo_pose_with_covariance.pose.covariance[35] = std_dev_yaw * std_dev_yaw;
  _pub->publish(_geo_pose_with_covariance);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(_node_name);

  node->declare_parameter("fix_only_publish", _fix_only_publish);
  node->declare_parameter("fix_judgement_type", _fix_judgement_type);
  node->declare_parameter("fix_std_pos_thres", _fix_std_pos_thres);

  node->get_parameter("fix_only_publish", _fix_only_publish);
  node->get_parameter("fix_judgement_type", _fix_judgement_type);
  node->get_parameter("fix_std_pos_thres", _fix_std_pos_thres);

  std::cout<< "fix_only_publish "<< _fix_only_publish<<std::endl;
  std::cout<< "fix_judgement_type "<< _fix_judgement_type<<std::endl;
  std::cout<< "fix_std_pos_thres "<< _fix_std_pos_thres<<std::endl;

  auto sub1 = node->create_subscription<eagleye_msgs::msg::Heading>("eagleye/heading_interpolate_3rd", 1000, heading_callback);
  auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>("eagleye/fix", 1000, fix_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Rolling>("eagleye/rolling", 1000, rolling_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Pitching>("eagleye/pitching", 1000, pitching_callback);
  _pub = node->create_publisher<geographic_msgs::msg::GeoPoseWithCovarianceStamped>("eagleye/geo_pose_with_covariance", 1000);
  rclcpp::spin(node);

  return 0;
}
