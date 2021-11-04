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
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/Position.h"
#include "tf/transform_broadcaster.h"
#include "coordinate/coordinate.hpp"


static eagleye_msgs::Heading eagleye_heading;
static eagleye_msgs::Position eagleye_position;
static geometry_msgs::Quaternion _quat;

static ros::Publisher pub;
static geometry_msgs::PoseStamped pose;

static double m_lat,m_lon,m_h;
static double m_x,m_y,m_z;
static int convert_height_num = 0;
static int plane = 7;
static int tf_num = 1;
static std::string parent_frame_id, child_frame_id;

static ConvertHeight convert_height;

void heading_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  eagleye_heading.header = msg->header;
  eagleye_heading.heading_angle = msg->heading_angle;
  eagleye_heading.status = msg->status;
}

void position_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  eagleye_position.header = msg->header;
  eagleye_position.enu_pos = msg->enu_pos;
  eagleye_position.ecef_base_pos = msg->ecef_base_pos;
  eagleye_position.status = msg->status;
}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  double llh[3] = {0};
  double _llh[3] = {0};
  double xyz[3] = {0};
  double geoid_height = 0;

  if (eagleye_position.status.enabled_status == true)
  {
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
  }

  if (eagleye_heading.status.enabled_status == true)
  {
    eagleye_heading.heading_angle = fmod(eagleye_heading.heading_angle,2*M_PI);
    _quat = tf::createQuaternionMsgFromYaw((90* M_PI / 180)-eagleye_heading.heading_angle);
  }
  else
  {
    _quat = tf::createQuaternionMsgFromYaw(0);
  }

  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = xyz[1];
  pose.pose.position.y = xyz[0];
  pose.pose.position.z = xyz[2];
  pose.pose.orientation = _quat;
  pub.publish(pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180)-eagleye_heading.heading_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, parent_frame_id, child_frame_id));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fix2pose");
  ros::NodeHandle n;

  n.getParam("fix2pose_node/plane",plane);
  n.getParam("fix2pose_node/tf_num",tf_num);
  n.getParam("fix2pose_node/convert_height_num",convert_height_num);
  n.getParam("fix2pose_node/parent_frame_id",parent_frame_id);
  n.getParam("fix2pose_node/child_frame_id",child_frame_id);

  std::cout<< "plane "<<plane<<std::endl;
  std::cout<< "tf_num "<<tf_num<<std::endl;
  std::cout<< "convert_height_num "<<convert_height_num<<std::endl;
  std::cout<< "parent_frame_id "<<parent_frame_id<<std::endl;
  std::cout<< "child_frame_id "<<child_frame_id<<std::endl;

  ros::Subscriber sub1 = n.subscribe("eagleye/heading_interpolate_3rd", 1000, heading_callback);
  ros::Subscriber sub2 = n.subscribe("eagleye/enu_absolute_pos_interpolate", 1000, position_callback);
  ros::Subscriber sub3 = n.subscribe("eagleye/fix", 1000, fix_callback);
  pub = n.advertise<geometry_msgs::PoseStamped>("/eagleye/pose", 1000);
  ros::spin();

  return 0;
}
