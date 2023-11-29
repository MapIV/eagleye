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
 * geo_pose_converter.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <eagleye_coordinate/eagleye_coordinate.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <llh_converter/llh_converter.hpp>

rclcpp::Clock _ros_clock(RCL_ROS_TIME);

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pub2;
std::shared_ptr<tf2_ros::TransformBroadcaster> _br;

static std::string _parent_frame_id, _child_frame_id;
static std::string _base_link_frame_id, _gnss_frame_id;

std::string geoid_file_path = ament_index_cpp::get_package_share_directory("llh_converter") + "/data/gsigeo2011_ver2_1.asc";
llh_converter::LLHConverter _lc(geoid_file_path);
llh_converter::LLHParam _llh_param;

std::string _node_name = "eagleye_geo_pose_converter";

tf2_ros::Buffer _tf_buffer(std::make_shared<rclcpp::Clock>(_ros_clock));

void geo_pose_callback(const geographic_msgs::msg::GeoPoseWithCovarianceStamped::ConstSharedPtr msg)
{
  double llh[3] = {0};
  double xyz[3] = {0};

  llh[0] = msg->pose.pose.position.latitude * M_PI / 180;
  llh[1] = msg->pose.pose.position.longitude* M_PI / 180;
  llh[2] = msg->pose.pose.position.altitude;

  _lc.convertRad2XYZ(llh[0], llh[1], llh[2], xyz[0], xyz[1], xyz[2], _llh_param);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = xyz[0];
  pose.pose.position.y = xyz[1];
  pose.pose.position.z = xyz[2];
  pose.pose.orientation = msg->pose.pose.orientation;

  const auto localization_quat = tf2::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);

  geometry_msgs::msg::PoseStamped::SharedPtr transformed_pose_msg_ptr(
    new geometry_msgs::msg::PoseStamped);

  geometry_msgs::msg::TransformStamped::SharedPtr TF_sensor_to_base_ptr(new geometry_msgs::msg::TransformStamped);
  try
  {
    *TF_sensor_to_base_ptr = _tf_buffer.lookupTransform(_gnss_frame_id, _base_link_frame_id, tf2::TimePointZero);

    tf2::Transform transform, transform2, transfrom3;
    transform.setOrigin(tf2::Vector3(pose.pose.position.x, pose.pose.position.y,
      pose.pose.position.z));
    transform.setRotation(localization_quat);
    tf2::Quaternion q2(TF_sensor_to_base_ptr->transform.rotation.x, TF_sensor_to_base_ptr->transform.rotation.y,
      TF_sensor_to_base_ptr->transform.rotation.z, TF_sensor_to_base_ptr->transform.rotation.w);
    transform2.setOrigin(tf2::Vector3(TF_sensor_to_base_ptr->transform.translation.x,
      TF_sensor_to_base_ptr->transform.translation.y, TF_sensor_to_base_ptr->transform.translation.z));
    transform2.setRotation(q2);
    transfrom3 = transform * transform2;

    pose.header.frame_id = _parent_frame_id;
    pose.pose.position.x = transfrom3.getOrigin().getX();
    pose.pose.position.y = transfrom3.getOrigin().getY();
    pose.pose.position.z = transfrom3.getOrigin().getZ();
    pose.pose.orientation.x = transfrom3.getRotation().getX();
    pose.pose.orientation.y = transfrom3.getRotation().getY();
    pose.pose.orientation.z = transfrom3.getRotation().getZ();
    pose.pose.orientation.w = transfrom3.getRotation().getW();

  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(rclcpp::get_logger(_node_name), "%s", ex.what());
    return;
  }

  _pub->publish(pose);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
  pose_with_covariance.header = pose.header;
  pose_with_covariance.pose.pose = pose.pose;

  // Covariance in NavSatFix is in ENU coordinate while the one in GeoPoseWithCovariance is in Lat/Lon/Alt coordinate.
  // In order to be consistent with the msg definition, we need to swap the covariance of x and y.
  pose_with_covariance.pose.covariance[0] = msg->pose.covariance[7];
  pose_with_covariance.pose.covariance[7] = msg->pose.covariance[0];
  pose_with_covariance.pose.covariance[14] = msg->pose.covariance[14];
  pose_with_covariance.pose.covariance[21] = msg->pose.covariance[21];
  pose_with_covariance.pose.covariance[28] = msg->pose.covariance[28];
  pose_with_covariance.pose.covariance[35] = msg->pose.covariance[35];
  _pub2->publish(pose_with_covariance);
 
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  // NOTE: currently geo_pose_fuser, the node before this node, ignores roll and pitch for robust estimation results.
  transform.setRotation(localization_quat);

  geometry_msgs::msg::TransformStamped trans_msg;
  trans_msg.header.stamp = msg->header.stamp;
  trans_msg.header.frame_id = _parent_frame_id;
  trans_msg.child_frame_id = _child_frame_id;
  trans_msg.transform = tf2::toMsg(transform);
  _br->sendTransform(trans_msg);
}

int main(int argc, char** argv)
{
  int plane = 7;
  int tf_num = 7;
  int convert_height_num = 7;
  int geoid_type = 0;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(_node_name);

  node->declare_parameter("plane", plane);
  node->declare_parameter("tf_num", tf_num);
  node->declare_parameter("convert_height_num", convert_height_num);
  node->declare_parameter("geoid_type", geoid_type);
  node->declare_parameter("parent_frame_id", _parent_frame_id);
  node->declare_parameter("child_frame_id", _child_frame_id);
  node->declare_parameter("base_link_frame_id", _base_link_frame_id);
  node->declare_parameter("gnss_frame_id", _gnss_frame_id);

  node->get_parameter("plane", plane);
  node->get_parameter("tf_num", tf_num);
  node->get_parameter("convert_height_num", convert_height_num);
  node->get_parameter("geoid_type", geoid_type);
  node->get_parameter("parent_frame_id", _parent_frame_id);
  node->get_parameter("child_frame_id", _child_frame_id);
  node->get_parameter("base_link_frame_id", _base_link_frame_id);
  node->get_parameter("gnss_frame_id", _gnss_frame_id);

  std::cout<< "plane "<< plane<<std::endl;
  std::cout<< "tf_num "<< tf_num<<std::endl;
  std::cout<< "convert_height_num "<< convert_height_num<<std::endl;
  std::cout<< "geoid_type "<< geoid_type<<std::endl;
  std::cout<< "parent_frame_id "<< _parent_frame_id<<std::endl;
  std::cout<< "child_frame_id "<< _child_frame_id<<std::endl;
  std::cout<< "base_link_frame_id "<< _base_link_frame_id<<std::endl;
  std::cout<< "gnss_frame_id "<< _gnss_frame_id<<std::endl;

  
  if (tf_num == 1)
  {
    _llh_param.use_mgrs = false;
    _llh_param.plane_num = plane;  
  }
  else if (tf_num == 2)
  {
    _llh_param.use_mgrs = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(_node_name), "tf_num is not valid");
    rclcpp::shutdown();
  }

  if (convert_height_num == 0)
  {
    RCLCPP_INFO(rclcpp::get_logger(_node_name), "convert_height_num is 0(no convert)");
  }
  else if (convert_height_num == 1)
  {
    _llh_param.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO; 
  }
  else if (convert_height_num == 2)
  {
    _llh_param.height_convert_type = llh_converter::ConvertType::ORTHO2ELLIPS;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(_node_name), "convert_height_num is not valid");
    rclcpp::shutdown();
  }

 if(geoid_type == 0)
  {
    _llh_param.geoid_type = llh_converter::GeoidType::EGM2008;
  }
  else if(geoid_type == 1)
  {
    _llh_param.geoid_type = llh_converter::GeoidType::GSIGEO2011; 
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(_node_name), "GeoidType is not valid");
    rclcpp::shutdown();
  }

  tf2_ros::TransformListener tf_listener(_tf_buffer);
  auto sub = node->create_subscription<geographic_msgs::msg::GeoPoseWithCovarianceStamped>("eagleye/geo_pose_with_covariance", 1000, geo_pose_callback);
  _pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("eagleye/pose", 1000);
  _pub2 = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("eagleye/pose_with_covariance", 1000);
  _br = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  rclcpp::spin(node);

  return 0;
}
