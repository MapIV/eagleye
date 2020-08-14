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

static double gps_x,gps_y,gps_z,gps_yaw,gps_pitch,gps_roll;
static double imu_x,imu_y,imu_z,imu_yaw,imu_pitch,imu_roll;

static double m_lat,m_lon,m_h;
static double m_x,m_y,m_z;
static bool altitude_estimate;
static int plane = 7;
static int tf_num = 1;

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
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));

  static tf::TransformBroadcaster br2;
  tf::Transform transform2;
  tf::Quaternion q2;
  transform2.setOrigin(tf::Vector3(gps_x, gps_y,gps_z));
  q2.setRPY(gps_yaw, gps_pitch, gps_roll);
  transform2.setRotation(q2);
  br2.sendTransform(tf::StampedTransform(transform2, msg->header.stamp, "gps", "base_link"));

  static tf::TransformBroadcaster br3;
  tf::Transform transform3;
  tf::Quaternion q3;
  transform3.setOrigin(tf::Vector3(imu_x, imu_y,imu_z));
  q3.setRPY(imu_yaw, imu_pitch, imu_roll);
  transform3.setRotation(q3);
  br3.sendTransform(tf::StampedTransform(transform3, msg->header.stamp, "base_link", "imu"));


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fix2pose");
  ros::NodeHandle n;

  n.getParam("plane",plane);
  n.getParam("tf_num",tf_num);
  n.getParam("altitude_estimate",altitude_estimate);
  n.getParam("gps_x",gps_x);
  n.getParam("gps_y",gps_y);
  n.getParam("gps_z",gps_z);
  n.getParam("gps_yaw",gps_yaw);
  n.getParam("gps_pitch",gps_pitch);
  n.getParam("gps_roll",gps_roll);
  n.getParam("imu_x",imu_x);
  n.getParam("imu_y",imu_y);
  n.getParam("imu_z",imu_z);
  n.getParam("imu_yaw",imu_yaw);
  n.getParam("imu_pitch",imu_pitch);
  n.getParam("imu_roll",imu_roll);

  std::cout<< "plane "<<plane<<std::endl;
  std::cout<< "tf_num "<<tf_num<<std::endl;
  std::cout<< "altitude_estimate "<<altitude_estimate<<std::endl;
  std::cout<< "gps_x "<<plane<<std::endl;
  std::cout<< "gps_y "<<tf_num<<std::endl;
  std::cout<< "gps_z "<<altitude_estimate<<std::endl;
  std::cout<< "gps_yaw "<<plane<<std::endl;
  std::cout<< "gps_pitch "<<tf_num<<std::endl;
  std::cout<< "gps_roll "<<altitude_estimate<<std::endl;
  std::cout<< "imu_x "<<plane<<std::endl;
  std::cout<< "imu_y "<<tf_num<<std::endl;
  std::cout<< "imu_z "<<altitude_estimate<<std::endl;
  std::cout<< "imu_yaw "<<plane<<std::endl;
  std::cout<< "imu_pitch "<<tf_num<<std::endl;
  std::cout<< "imu_roll "<<altitude_estimate<<std::endl;


  ros::Subscriber sub1 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, heading_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos_interpolate", 1000, position_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/fix", 1000, fix_callback);
  pub = n.advertise<geometry_msgs::PoseStamped>("/eagleye/pose", 1000);
  ros::spin();

  return 0;
}
