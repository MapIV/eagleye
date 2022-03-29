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
 * position.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::Distance distance;
static eagleye_msgs::Heading heading_interpolate_3rd;
static eagleye_msgs::Position enu_absolute_pos;
static geometry_msgs::Vector3Stamped enu_vel;
static sensor_msgs::NavSatFix fix;
static ros::Publisher pub;

struct PositionParameter position_parameter;
struct PositionStatus position_status;

static std::string use_gnss_mode;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_3rd = *msg;
}

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  fix = *msg;
}

void timer_callback(const ros::TimerEvent& e, tf2_ros::TransformListener* tfListener_, tf2_ros::Buffer* tfBuffer_)
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_->lookupTransform(position_parameter.tf_gnss_parent_flame, position_parameter.tf_gnss_child_flame, ros::Time(0));

    position_parameter.tf_gnss_translation_x = transformStamped.transform.translation.x;
    position_parameter.tf_gnss_translation_y = transformStamped.transform.translation.y;
    position_parameter.tf_gnss_translation_z = transformStamped.transform.translation.z;
    position_parameter.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
    position_parameter.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
    position_parameter.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
    position_parameter.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.header.frame_id = "base_link";

  if (use_gnss_mode == "rtklib" || use_gnss_mode == "RTKLIB") // use RTKLIB mode
    position_estimate(rtklib_nav, velocity_scale_factor, distance, heading_interpolate_3rd, enu_vel, position_parameter, &position_status, &enu_absolute_pos);
  else if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
    position_estimate(fix, velocity_scale_factor, distance, heading_interpolate_3rd, enu_vel, position_parameter, &position_status, &enu_absolute_pos);
  
  if(enu_absolute_pos.status.estimate_status == true)
  {
    pub.publish(enu_absolute_pos);
  }
  enu_absolute_pos.status.estimate_status = false;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position");
  ros::NodeHandle n;

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_navsatfix_topic_name = "/navsat/fix";

  n.getParam("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("navsatfix_topic",subscribe_navsatfix_topic_name);
  n.getParam("position/estimated_distance",position_parameter.estimated_distance);
  n.getParam("position/separation_distance",position_parameter.separation_distance);
  n.getParam("position/estimated_velocity_threshold",position_parameter.estimated_velocity_threshold);
  n.getParam("position/outlier_threshold",position_parameter.outlier_threshold);
  n.getParam("position/estimated_enu_vel_coefficient",position_parameter.estimated_enu_vel_coefficient);
  n.getParam("position/estimated_position_coefficient",position_parameter.estimated_position_coefficient);
  n.getParam("ecef_base_pos/x",position_parameter.ecef_base_pos_x);
  n.getParam("ecef_base_pos/y",position_parameter.ecef_base_pos_y);
  n.getParam("ecef_base_pos/z",position_parameter.ecef_base_pos_z);
  n.getParam("tf_gnss_flame/parent", position_parameter.tf_gnss_parent_flame);
  n.getParam("tf_gnss_flame/child", position_parameter.tf_gnss_child_flame);
  n.getParam("use_gnss_mode",use_gnss_mode);

  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;
  std::cout<< "estimated_distance "<<position_parameter.estimated_distance<<std::endl;
  std::cout<< "separation_distance "<<position_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<position_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<position_parameter.outlier_threshold<<std::endl;
  std::cout<< "estimated_enu_vel_coefficient "<<position_parameter.estimated_enu_vel_coefficient<<std::endl;
  std::cout<< "estimated_position_coefficient "<<position_parameter.estimated_position_coefficient<<std::endl;
  std::cout<< "tf_gnss_flame/parent "<<position_parameter.tf_gnss_parent_flame<<std::endl;
  std::cout<< "tf_gnss_flame/child "<<position_parameter.tf_gnss_child_flame<<std::endl;
  std::cout<< "use_gnss_mode "<<use_gnss_mode<<std::endl;

  ros::Subscriber sub1 = n.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe(subscribe_navsatfix_topic_name, 1000, fix_callback, ros::TransportHints().tcpNoDelay());
  
  pub = n.advertise<eagleye_msgs::Position>("enu_absolute_pos", 1000);

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_(tfBuffer_);
  ros::Timer timer = n.createTimer(ros::Duration(0.5), boost::bind(timer_callback,_1, &tfListener_, &tfBuffer_));

  ros::spin();

  return 0;
}
