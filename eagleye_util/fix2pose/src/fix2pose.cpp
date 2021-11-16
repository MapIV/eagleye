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
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "eagleye_msgs/msg/heading.hpp"
#include "eagleye_msgs/msg/position.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "coordinate/coordinate.hpp"

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}


static eagleye_msgs::msg::Heading eagleye_heading;
static eagleye_msgs::msg::Position eagleye_position;
static geometry_msgs::msg::Quaternion _quat;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> br;
std::shared_ptr<tf2_ros::TransformBroadcaster> br2;
static geometry_msgs::msg::PoseStamped pose;

static double gps_x,gps_y,gps_z,gps_yaw,gps_pitch,gps_roll;
static double imu_x,imu_y,imu_z,imu_yaw,imu_pitch,imu_roll;

static double m_lat,m_lon,m_h;
static double m_x,m_y,m_z;
static bool altitude_estimate;
static int plane = 7;
static int tf_num = 1;

void heading_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  eagleye_heading.header = msg->header;
  eagleye_heading.heading_angle = msg->heading_angle;
  eagleye_heading.status = msg->status;
}

void position_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  eagleye_position.header = msg->header;
  eagleye_position.enu_pos = msg->enu_pos;
  eagleye_position.ecef_base_pos = msg->ecef_base_pos;
  eagleye_position.status = msg->status;
}

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  double llh[3] = {0};
  double _llh[3] = {0};
  double xyz[3] = {0};
  double height = 0;

  if (eagleye_position.status.enabled_status == true)
  {
    llh[0] = msg->latitude * M_PI / 180;
    llh[1] = msg->longitude* M_PI / 180;
    llh[2] = msg->altitude;

    if (altitude_estimate == true)
    {
      _llh[0] = llh[0] * 180/M_PI;
      _llh[1] = llh[1] * 180/M_PI;
      _llh[2] = llh[2];
      hgeoid(_llh,&height);
      llh[2] = llh[2] - height;
    }

    if (tf_num == 1)
    {
      ll2xy(plane,llh,xyz);
    }
    else if (tf_num == 2)
    {
      ll2xy_mgrs(llh,xyz);
    }
  }

  if (eagleye_heading.status.enabled_status == true)
  {
    eagleye_heading.heading_angle = fmod(eagleye_heading.heading_angle,2*M_PI);
    _quat = createQuaternionMsgFromYaw((90* M_PI / 180)-eagleye_heading.heading_angle);
  }
  else
  {
    _quat = createQuaternionMsgFromYaw(0);
  }

  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = xyz[1];
  pose.pose.position.y = xyz[0];
  pose.pose.position.z = xyz[2];
  pose.pose.orientation = _quat;
  pub->publish(pose);
  
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180)-eagleye_heading.heading_angle);
  transform.setRotation(q);

  // geometry_msgs::msg::TransformStamped trans_msg;
  // trans_msg.header.stamp = msg->header.stamp;
  // trans_msg.header.frame_id = "map";
  // trans_msg.child_frame_id = "eagleye";
  // trans_msg.transform = tf2::toMsg(transform);
  // br->sendTransform(trans_msg);

  tf2::Transform transform2;
  tf2::Quaternion q2;
  transform2.setOrigin(transform*tf2::Vector3(-gps_x, -gps_y,-gps_z));
  q2.setRPY(gps_roll, gps_pitch, gps_yaw);
  transform2.setRotation(transform*q2);

  geometry_msgs::msg::TransformStamped trans_msg2;
  trans_msg2.header.stamp = msg->header.stamp;
  trans_msg2.header.frame_id = "map";
  trans_msg2.child_frame_id = "base_link";
  trans_msg2.transform = tf2::toMsg(transform2);
  br2->sendTransform(trans_msg2);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fix2pose");

  node->declare_parameter("plane",plane);
  node->declare_parameter("tf_num",tf_num);
  node->declare_parameter("altitude_estimate",altitude_estimate);
  node->declare_parameter("gps_x",gps_x);
  node->declare_parameter("gps_y",gps_y);
  node->declare_parameter("gps_z",gps_z);
  node->declare_parameter("gps_yaw",gps_yaw);
  node->declare_parameter("gps_pitch",gps_pitch);
  node->declare_parameter("gps_roll",gps_roll);
  node->declare_parameter("imu_x",imu_x);
  node->declare_parameter("imu_y",imu_y);
  node->declare_parameter("imu_z",imu_z);
  node->declare_parameter("imu_yaw",imu_yaw);
  node->declare_parameter("imu_pitch",imu_pitch);
  node->declare_parameter("imu_roll",imu_roll);

  node->get_parameter("plane",plane);
  node->get_parameter("tf_num",tf_num);
  node->get_parameter("altitude_estimate",altitude_estimate);
  node->get_parameter("gps_x",gps_x);
  node->get_parameter("gps_y",gps_y);
  node->get_parameter("gps_z",gps_z);
  node->get_parameter("gps_yaw",gps_yaw);
  node->get_parameter("gps_pitch",gps_pitch);
  node->get_parameter("gps_roll",gps_roll);
  node->get_parameter("imu_x",imu_x);
  node->get_parameter("imu_y",imu_y);
  node->get_parameter("imu_z",imu_z);
  node->get_parameter("imu_yaw",imu_yaw);
  node->get_parameter("imu_pitch",imu_pitch);
  node->get_parameter("imu_roll",imu_roll);


  std::cout<< "plane "<<plane<<std::endl;
  std::cout<< "tf_num "<<tf_num<<std::endl;
  std::cout<< "altitude_estimate "<<altitude_estimate<<std::endl;
  std::cout<< "gps_x "<<gps_x<<std::endl;
  std::cout<< "gps_y "<<gps_y<<std::endl;
  std::cout<< "gps_z "<<gps_z<<std::endl;
  std::cout<< "gps_yaw "<<gps_yaw<<std::endl;
  std::cout<< "gps_pitch "<<gps_pitch<<std::endl;
  std::cout<< "gps_roll "<<gps_roll<<std::endl;
  std::cout<< "imu_x "<<imu_x<<std::endl;
  std::cout<< "imu_y "<<imu_y<<std::endl;
  std::cout<< "imu_z "<<imu_z<<std::endl;
  std::cout<< "imu_yaw "<<imu_yaw<<std::endl;
  std::cout<< "imu_pitch "<<imu_pitch<<std::endl;
  std::cout<< "imu_roll "<<imu_roll<<std::endl;

  auto sub1 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_interpolate_3rd", 1000, heading_callback);
  auto sub2 = node->create_subscription<eagleye_msgs::msg::Position>("/eagleye/enu_absolute_pos_interpolate", 1000, position_callback);
  auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>("/eagleye/fix", 1000, fix_callback);
  pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/eagleye/pose", 1000);
  br = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  br2 = std::make_shared<tf2_ros::TransformBroadcaster>(node, 100);
  rclcpp::spin(node);

  return 0;
}
