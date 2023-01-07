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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "eagleye_coordinate/eagleye_coordinate.hpp"

rclcpp::Clock _ros_clock(RCL_ROS_TIME);

static eagleye_msgs::msg::Rolling _eagleye_rolling;
static eagleye_msgs::msg::Pitching _eagleye_pitching;
static eagleye_msgs::msg::Heading _eagleye_heading;
static geometry_msgs::msg::Quaternion _quat;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pub2;
std::shared_ptr<tf2_ros::TransformBroadcaster> _br;
std::shared_ptr<tf2_ros::TransformBroadcaster> _br2;
static geometry_msgs::msg::PoseStamped _pose;
static geometry_msgs::msg::PoseWithCovarianceStamped _pose_with_covariance;

static int _convert_height_num = 0;
static int _plane = 7;
static int _tf_num = 1;
static std::string _parent_frame_id, _child_frame_id;
static std::string _base_link_frame_id, _gnss_frame_id;

static ConvertHeight _convert_height;

bool _fix_only_publish = false;

std::string node_name = "fix2pose";

tf2_ros::Buffer _tf_buffer(std::make_shared<rclcpp::Clock>(_ros_clock));

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
  bool fix_only_publish_flag = (msg->position_covariance[0] < 0.01 && _eagleye_heading.status.enabled_status);
  // std::cout<< "fix_only_publish_flag "<< fix_only_publish_flag<<std::endl;
  if(_fix_only_publish && !fix_only_publish_flag)
  {
    return;
  }

  double llh[3] = {0};
  double xyz[3] = {0};

  llh[0] = msg->latitude * M_PI / 180;
  llh[1] = msg->longitude* M_PI / 180;
  llh[2] = msg->altitude;

  if (_convert_height_num == 1)
  {
    _convert_height.setLLH(msg->latitude,msg->longitude,msg->altitude);
    llh[2] = _convert_height.convert2altitude();
  }
  else if(_convert_height_num == 2)
  {
    _convert_height.setLLH(msg->latitude,msg->longitude,msg->altitude);
    llh[2] = _convert_height.convert2ellipsoid();
  }

  if (_tf_num == 1)
  {
    ll2xy(_plane,llh,xyz);
  }
  else if (_tf_num == 2)
  {
    ll2xy_mgrs(llh,xyz);
  }

  tf2::Quaternion localization_quat;
  if (_eagleye_heading.status.enabled_status)
  {
    _eagleye_heading.heading_angle = fmod(_eagleye_heading.heading_angle,2*M_PI);
    localization_quat.setRPY(_eagleye_rolling.rolling_angle, _eagleye_pitching.pitching_angle, (90* M_PI / 180)-_eagleye_heading.heading_angle);
  }
  else
  {
    tf2::Quaternion localization_quat;
    tf2::Matrix3x3(localization_quat).setRPY(0, 0, 0);
  }
  _quat = tf2::toMsg(localization_quat);

  geometry_msgs::msg::PoseStamped::Ptr transformed_pose_msg_ptr(
    new geometry_msgs::msg::PoseStamped);

  if(_fix_only_publish)
  {
    geometry_msgs::msg::TransformStamped::Ptr TF_sensor_to_base_ptr(new geometry_msgs::msg::TransformStamped);
    try
    {
      *TF_sensor_to_base_ptr = _tf_buffer.lookupTransform(_base_link_frame_id, _gnss_frame_id, tf2::TimePointZero);

      tf2::doTransform(_pose, *transformed_pose_msg_ptr, *TF_sensor_to_base_ptr);
      std::string map_frame = "map";
      transformed_pose_msg_ptr->header = _pose.header;
      transformed_pose_msg_ptr->header.frame_id = _parent_frame_id;
      _pose = *transformed_pose_msg_ptr;
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(rclcpp::get_logger(node_name), "%s", ex.what());
      return;
    }
  }

  _pose.header = msg->header;
  _pose.header.frame_id = "map";
  _pose.pose.position.x = xyz[1];
  _pose.pose.position.y = xyz[0];
  _pose.pose.position.z = xyz[2];
  _pose.pose.orientation = _quat;
  _pub->publish(_pose);

  _pose_with_covariance.header = _pose.header;
  _pose_with_covariance.pose.pose = _pose.pose;
  // TODO(Map IV): temporary value
  double std_dev_roll = 100; // [rad]
  double std_dev_pitch = 100; // [rad]
  double std_dev_yaw = 100; // [rad]
  if(_eagleye_rolling.status.enabled_status) std_dev_roll = 0.5 / 180 * M_PI;
  if(_eagleye_pitching.status.enabled_status) std_dev_pitch = 0.5 / 180 * M_PI;
  if(_eagleye_heading.status.enabled_status) std_dev_yaw = 0.2 / 180 * M_PI;
  _pose_with_covariance.pose.covariance[0] = msg->position_covariance[0];
  _pose_with_covariance.pose.covariance[7] = msg->position_covariance[4];
  _pose_with_covariance.pose.covariance[14] = msg->position_covariance[8];
  _pose_with_covariance.pose.covariance[21] = std_dev_roll * std_dev_roll;
  _pose_with_covariance.pose.covariance[28] = std_dev_pitch * std_dev_pitch;
  _pose_with_covariance.pose.covariance[35] = std_dev_yaw * std_dev_yaw;
  _pub2->publish(_pose_with_covariance);
  
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180)- _eagleye_heading.heading_angle);
  transform.setRotation(q);

  geometry_msgs::msg::TransformStamped trans_msg;
  trans_msg.header.stamp = msg->header.stamp;
  trans_msg.header.frame_id = _parent_frame_id;
  trans_msg.child_frame_id = _child_frame_id;
  trans_msg.transform = tf2::toMsg(transform);
  _br->sendTransform(trans_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(node_name);

  node->declare_parameter("plane", _plane);
  node->declare_parameter("tf_num", _tf_num);
  node->declare_parameter("convert_height_num", _convert_height_num);
  node->declare_parameter("parent_frame_id", _parent_frame_id);
  node->declare_parameter("child_frame_id", _child_frame_id);
  node->declare_parameter("fix_only_publish", _fix_only_publish);
  node->declare_parameter("base_link_frame_id", _base_link_frame_id);
  node->declare_parameter("gnss_frame_id", _gnss_frame_id);

  node->get_parameter("plane", _plane);
  node->get_parameter("tf_num", _tf_num);
  node->get_parameter("convert_height_num", _convert_height_num);
  node->get_parameter("parent_frame_id", _parent_frame_id);
  node->get_parameter("child_frame_id", _child_frame_id);
  node->get_parameter("fix_only_publish", _fix_only_publish);
  node->get_parameter("base_link_frame_id", _base_link_frame_id);
  node->get_parameter("gnss_frame_id", _gnss_frame_id);

  std::cout<< "plane "<< _plane<<std::endl;
  std::cout<< "tf_num "<< _tf_num<<std::endl;
  std::cout<< "convert_height_num "<< _convert_height_num<<std::endl;
  std::cout<< "parent_frame_id "<< _parent_frame_id<<std::endl;
  std::cout<< "child_frame_id "<< _child_frame_id<<std::endl;
  std::cout<< "fix_only_publish "<< _fix_only_publish<<std::endl;
  std::cout<< "base_link_frame_id "<< _base_link_frame_id<<std::endl;
  std::cout<< "gnss_frame_id "<< _gnss_frame_id<<std::endl;

  tf2_ros::TransformListener tf_listener(_tf_buffer);
  auto sub1 = node->create_subscription<eagleye_msgs::msg::Heading>("eagleye/heading_interpolate_3rd", 1000, heading_callback);
  auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>("eagleye/fix", 1000, fix_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Rolling>("eagleye/rolling", 1000, rolling_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Pitching>("eagleye/pitching", 1000, pitching_callback);
  _pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("eagleye/pose", 1000);
  _pub2 = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("eagleye/pose_with_covariance", 1000);
  _br = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  _br2 = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  rclcpp::spin(node);

  return 0;
}
