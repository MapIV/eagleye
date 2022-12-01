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
 * fix2pose.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "eagleye_msgs/msg/rolling.hpp"
#include "eagleye_msgs/msg/pitching.hpp"
#include "eagleye_msgs/msg/heading.hpp"
#include "eagleye_msgs/msg/position.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "eagleye_coordinate/eagleye_coordinate.hpp"

static eagleye_msgs::msg::Rolling eagleye_rolling;
static eagleye_msgs::msg::Pitching eagleye_pitching;
static eagleye_msgs::msg::Heading eagleye_heading;
static eagleye_msgs::msg::Position eagleye_position;
static geometry_msgs::msg::Quaternion _quat;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub2;
std::shared_ptr<tf2_ros::TransformBroadcaster> br;
std::shared_ptr<tf2_ros::TransformBroadcaster> br2;
static geometry_msgs::msg::PoseStamped pose;
static geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;

static int convert_height_num = 0;
static int plane = 7;
static int tf_num = 1;
static std::string parent_frame_id, child_frame_id;

static ConvertHeight convert_height;

void heading_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  eagleye_heading = *msg;
}

void rolling_callback(const eagleye_msgs::msg::Rolling::ConstSharedPtr msg)
{
  eagleye_rolling = *msg;
}

void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
{
  eagleye_pitching = *msg;
}

void position_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  eagleye_position = *msg;
}

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  double llh[3] = {0};
  double xyz[3] = {0};

  llh[0] = msg->latitude * M_PI / 180;
  llh[1] = msg->longitude* M_PI / 180;
  llh[2] = msg->altitude;

  if (convert_height_num == 1)
  {
    convert_height.setLLH(msg->latitude,msg->longitude,msg->altitude);
    llh[2] = convert_height.convert2altitude();
  }
  else if(convert_height_num == 2)
  {
    convert_height.setLLH(msg->latitude,msg->longitude,msg->altitude);
    llh[2] = convert_height.convert2ellipsoid();
  }

  if (tf_num == 1)
  {
    ll2xy(plane,llh,xyz);
  }
  else if (tf_num == 2)
  {
    ll2xy_mgrs(llh,xyz);
  }

  tf2::Quaternion localization_quat;
  if (eagleye_heading.status.enabled_status)
  {
    eagleye_heading.heading_angle = fmod(eagleye_heading.heading_angle,2*M_PI);
    localization_quat.setRPY(eagleye_rolling.rolling_angle,eagleye_pitching.pitching_angle,(90* M_PI / 180)-eagleye_heading.heading_angle);
  }
  else
  {
    tf2::Quaternion localization_quat;
    tf2::Matrix3x3(localization_quat).setRPY(0, 0, 0);
  }
  _quat = tf2::toMsg(localization_quat);

  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = xyz[1];
  pose.pose.position.y = xyz[0];
  pose.pose.position.z = xyz[2];
  pose.pose.orientation = _quat;
  pub->publish(pose);

  pose_with_covariance.header = pose.header;
  pose_with_covariance.pose.pose = pose.pose;
  // TODO(Map IV): temporary value
  double std_dev_roll = 100; // [rad]
  double std_dev_pitch = 100; // [rad]
  double std_dev_yaw = 100; // [rad]
  if(eagleye_rolling.status.enabled_status) std_dev_roll = 0.5 / 180 * M_PI;
  if(eagleye_pitching.status.enabled_status) std_dev_pitch = 0.5 / 180 * M_PI;
  if(eagleye_heading.status.enabled_status) std_dev_yaw = 0.2 / 180 * M_PI;
  pose_with_covariance.pose.covariance[0] = msg->position_covariance[0];
  pose_with_covariance.pose.covariance[7] = msg->position_covariance[4];
  pose_with_covariance.pose.covariance[14] = msg->position_covariance[8];
  pose_with_covariance.pose.covariance[21] = std_dev_roll * std_dev_roll;
  pose_with_covariance.pose.covariance[28] = std_dev_pitch * std_dev_pitch;
  pose_with_covariance.pose.covariance[35] = std_dev_yaw * std_dev_yaw;
  pub2->publish(pose_with_covariance);
  
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180)-eagleye_heading.heading_angle);
  transform.setRotation(q);

  geometry_msgs::msg::TransformStamped trans_msg;
  trans_msg.header.stamp = msg->header.stamp;
  trans_msg.header.frame_id = parent_frame_id;
  trans_msg.child_frame_id = child_frame_id;
  trans_msg.transform = tf2::toMsg(transform);
  br->sendTransform(trans_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fix2pose");

  node->declare_parameter("plane",plane);
  node->declare_parameter("tf_num",tf_num);
  node->declare_parameter("convert_height_num",convert_height_num);
  node->declare_parameter("parent_frame_id",parent_frame_id);
  node->declare_parameter("child_frame_id",child_frame_id);

  node->get_parameter("plane",plane);
  node->get_parameter("tf_num",tf_num);
  node->get_parameter("convert_height_num",convert_height_num);
  node->get_parameter("parent_frame_id",parent_frame_id);
  node->get_parameter("child_frame_id",child_frame_id);

  std::cout<< "plane"<<plane<<std::endl;
  std::cout<< "tf_num"<<tf_num<<std::endl;
  std::cout<< "convert_height_num"<<convert_height_num<<std::endl;
  std::cout<< "parent_frame_id"<<parent_frame_id<<std::endl;
  std::cout<< "child_frame_id"<<child_frame_id<<std::endl;

  auto sub1 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_interpolate_3rd", 1000, heading_callback);
  auto sub2 = node->create_subscription<eagleye_msgs::msg::Position>("/eagleye/enu_absolute_pos_interpolate", 1000, position_callback);
  auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>("/eagleye/fix", 1000, fix_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Rolling>("/eagleye/rolling", 1000, rolling_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Pitching>("/eagleye/pitching", 1000, pitching_callback);
  pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/eagleye/pose", 1000);
  pub2 = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/eagleye/pose_with_covariance", 1000);
  br = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  br2 = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  rclcpp::spin(node);

  return 0;
}
