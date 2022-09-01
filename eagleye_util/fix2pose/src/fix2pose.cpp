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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "eagleye_msgs/Rolling.h"
#include "eagleye_msgs/Pitching.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/Position.h"
#include "tf/transform_broadcaster.h"
#include "coordinate/coordinate.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/bind.hpp>

static eagleye_msgs::Rolling _eagleye_rolling;
static eagleye_msgs::Pitching _eagleye_pitching;
static eagleye_msgs::Heading _eagleye_heading;
static geometry_msgs::Quaternion _quat;

static ros::Publisher _pose_pub, _pose_with_covariance_pub;
static geometry_msgs::PoseStamped _pose;
static geometry_msgs::PoseWithCovarianceStamped _pose_with_covariance;

static int _convert_height_num = 0;
static int _plane = 7;
static int _tf_num = 1;
static std::string _parent_frame_id, _child_frame_id, _base_link_frame_id, _gnss_frame_id;

static ConvertHeight _convert_height;

bool _fix_only_publish = false;

void heading_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _eagleye_heading = *msg;
}

void rolling_callback(const eagleye_msgs::Rolling::ConstPtr& msg)
{
  _eagleye_rolling = *msg;
}

void pitching_callback(const eagleye_msgs::Pitching::ConstPtr& msg)
{
  _eagleye_pitching = *msg;
}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg, tf2_ros::TransformListener* tf_listener, tf2_ros::Buffer* tf_buffer)
{
  if(_fix_only_publish && msg->status.status != 0)
  {
    return;
  }

  double llh[3] = {0};
  double xyz[3] = {0};
  double geoid_height = 0;

  llh[0] = msg->latitude * M_PI / 180;
  llh[1] = msg->longitude* M_PI / 180;
  llh[2] = msg->altitude;

  if (_convert_height_num == 1)
  {
    _convert_height.setLLH(msg->latitude, msg->longitude, msg->altitude);
    llh[2] = _convert_height.convert2altitude();
  }
  else if(_convert_height_num == 2)
  {
    _convert_height.setLLH(msg->latitude, msg->longitude, msg->altitude);
    llh[2] = _convert_height.convert2ellipsoid();
  }

  if (_tf_num == 1)
  {
    ll2xy(_plane, llh, xyz);
  }
  else if (_tf_num == 2)
  {
    ll2xy_mgrs(llh, xyz);
  }

  if (_eagleye_heading.status.enabled_status == true)
  {
    _eagleye_heading.heading_angle = fmod(_eagleye_heading.heading_angle, 2*M_PI);
    tf::Quaternion tf_quat = tf::createQuaternionFromRPY(_eagleye_rolling.rolling_angle,
      _eagleye_pitching.pitching_angle, (90* M_PI / 180) - _eagleye_heading.heading_angle);
    quaternionTFToMsg(tf_quat, _quat);
  }
  else
  {
    _quat = tf::createQuaternionMsgFromYaw(0);
  }

  _pose.header = msg->header;
  _pose.header.frame_id = "map";
  _pose.pose.position.x = xyz[1];
  _pose.pose.position.y = xyz[0];
  _pose.pose.position.z = xyz[2];
  _pose.pose.orientation = _quat;

  geometry_msgs::PoseStamped::Ptr transformed_pose_msg_ptr(
    new geometry_msgs::PoseStamped);

  if(_fix_only_publish)
  {
    geometry_msgs::TransformStamped::Ptr TF_sensor_to_base_ptr(new geometry_msgs::TransformStamped);
    try
    {
      *TF_sensor_to_base_ptr = tf_buffer->lookupTransform(_base_link_frame_id, _gnss_frame_id, ros::Time(0));

      tf2::doTransform(_pose, *transformed_pose_msg_ptr, *TF_sensor_to_base_ptr);
      std::string map_frame = "map";
      transformed_pose_msg_ptr->header = _pose.header;
      transformed_pose_msg_ptr->header.frame_id = _parent_frame_id;
      _pose = *transformed_pose_msg_ptr;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }

  _pose_pub.publish(_pose);

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
  _pose_with_covariance_pub.publish(_pose_with_covariance);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180) - _eagleye_heading.heading_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, _parent_frame_id, _child_frame_id));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fix2pose");
  ros::NodeHandle nh;

  nh.getParam("fix2pose_node/plane", _plane);
  nh.getParam("fix2pose_node/tf_num", _tf_num);
  nh.getParam("fix2pose_node/convert_height_num", _convert_height_num);
  nh.getParam("fix2pose_node/parent_frame_id", _parent_frame_id);
  nh.getParam("fix2pose_node/child_frame_id", _child_frame_id);
  nh.getParam("fix2pose_node/fix_only_publish", _fix_only_publish);
  nh.getParam("fix2pose_node/base_link_frame_id", _base_link_frame_id);
  nh.getParam("fix2pose_node/gnss_frame_id", _gnss_frame_id);

  std::cout<< "plane " << _plane << std::endl;
  std::cout<< "tf_num " << _tf_num << std::endl;
  std::cout<< "convert_height_num " << _convert_height_num << std::endl;
  std::cout<< "parent_frame_id " << _parent_frame_id << std::endl;
  std::cout<< "child_frame_id " << _child_frame_id << std::endl;
  std::cout<< "fix_only_publish " << _fix_only_publish << std::endl;
  std::cout<< "base_link_frame_id " << _base_link_frame_id << std::endl;
  std::cout<< "gnss_frame_id " << _gnss_frame_id << std::endl;

  std::string fix_name;
  if(!_fix_only_publish)
  {
    fix_name = "eagleye/fix";
  }
  else {
    fix_name = "navsat/fix";
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Subscriber sub1 = nh.subscribe("eagleye/heading_interpolate_3rd", 1000, heading_callback);
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::NavSatFix>(fix_name, 1000, boost::bind(fix_callback,_1, &tf_listener, &tf_buffer));
  ros::Subscriber sub3 = nh.subscribe("eagleye/rolling", 1000, rolling_callback);
  ros::Subscriber sub4 = nh.subscribe("eagleye/pitching", 1000, pitching_callback);
  _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/eagleye/pose", 1000);
  _pose_with_covariance_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/eagleye/pose_with_covariance", 1000);
  ros::spin();

  return 0;
}
