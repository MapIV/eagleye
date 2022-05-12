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

static rtklib_msgs::RtklibNav _rtklib_nav;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
static eagleye_msgs::Distance _distance;
static eagleye_msgs::Heading _heading_interpolate_3rd;
static eagleye_msgs::Position _enu_absolute_pos;
static geometry_msgs::Vector3Stamped _enu_vel;
static nmea_msgs::Gpgga _gga;
static ros::Publisher _pub;

struct PositionParameter _position_parameter;
struct PositionStatus _position_status;

static std::string _use_gnss_mode;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  _rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _velocity = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  _distance = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_3rd = *msg;
}

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  _gga = *msg;
}

void timer_callback(const ros::TimerEvent& e, tf2_ros::TransformListener* tf_listener, tf2_ros::Buffer* tf_buffer)
{
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer->lookupTransform(_position_parameter.tf_gnss_parent_frame, _position_parameter.tf_gnss_child_frame, ros::Time(0));

    _position_parameter.tf_gnss_translation_x = transform_stamped.transform.translation.x;
    _position_parameter.tf_gnss_translation_y = transform_stamped.transform.translation.y;
    _position_parameter.tf_gnss_translation_z = transform_stamped.transform.translation.z;
    _position_parameter.tf_gnss_rotation_x = transform_stamped.transform.rotation.x;
    _position_parameter.tf_gnss_rotation_y = transform_stamped.transform.rotation.y;
    _position_parameter.tf_gnss_rotation_z = transform_stamped.transform.rotation.z;
    _position_parameter.tf_gnss_rotation_w = transform_stamped.transform.rotation.w;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  _enu_vel = *msg;
  _enu_absolute_pos.header = msg->header;
  _enu_absolute_pos.header.frame_id = "base_link";

  if (_use_gnss_mode == "rtklib" || _use_gnss_mode == "RTKLIB") // use RTKLIB mode
    position_estimate(_rtklib_nav, _velocity, _velocity_scale_factor, _distance, _heading_interpolate_3rd,
                      _enu_vel, _position_parameter, &_position_status, &_enu_absolute_pos);
  else if (_use_gnss_mode == "nmea" || _use_gnss_mode == "NMEA") // use NMEA mode
    position_estimate(_gga, _velocity, _velocity_scale_factor, _distance, _heading_interpolate_3rd,
                      _enu_vel, _position_parameter, &_position_status, &_enu_absolute_pos);
  
  if(_enu_absolute_pos.status.estimate_status)
  {
    _pub.publish(_enu_absolute_pos);
  }
  _enu_absolute_pos.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position");
  ros::NodeHandle nh;

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_gga_topic_name = "/navsat/gga";

  nh.getParam("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  nh.getParam("gga_topic",subscribe_gga_topic_name);
  nh.getParam("position/estimated_distance",_position_parameter.estimated_distance);
  nh.getParam("position/separation_distance",_position_parameter.separation_distance);
  nh.getParam("position/estimated_velocity_threshold",_position_parameter.estimated_velocity_threshold);
  nh.getParam("position/outlier_threshold",_position_parameter.outlier_threshold);
  nh.getParam("position/estimated_enu_vel_coefficient",_position_parameter.estimated_enu_vel_coefficient);
  nh.getParam("position/estimated_position_coefficient",_position_parameter.estimated_position_coefficient);
  nh.getParam("ecef_base_pos/x",_position_parameter.ecef_base_pos_x);
  nh.getParam("ecef_base_pos/y",_position_parameter.ecef_base_pos_y);
  nh.getParam("ecef_base_pos/z",_position_parameter.ecef_base_pos_z);
  nh.getParam("tf_gnss_frame/parent", _position_parameter.tf_gnss_parent_frame);
  nh.getParam("tf_gnss_frame/child", _position_parameter.tf_gnss_child_frame);
  nh.getParam("use_gnss_mode",_use_gnss_mode);

  std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;
  std::cout<< "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;
  std::cout<< "estimated_distance " << _position_parameter.estimated_distance << std::endl;
  std::cout<< "separation_distance " << _position_parameter.separation_distance << std::endl;
  std::cout<< "estimated_velocity_threshold " << _position_parameter.estimated_velocity_threshold << std::endl;
  std::cout<< "outlier_threshold " << _position_parameter.outlier_threshold << std::endl;
  std::cout<< "estimated_enu_vel_coefficient " << _position_parameter.estimated_enu_vel_coefficient << std::endl;
  std::cout<< "estimated_position_coefficient " << _position_parameter.estimated_position_coefficient << std::endl;
  std::cout<< "tf_gnss_frame/parent " << _position_parameter.tf_gnss_parent_frame << std::endl;
  std::cout<< "tf_gnss_frame/child " << _position_parameter.tf_gnss_child_frame << std::endl;
  std::cout<< "use_gnss_mode " << _use_gnss_mode << std::endl;

  ros::Subscriber sub1 = nh.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("velocity", 1000, velocity_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  
  _pub = nh.advertise<eagleye_msgs::Position>("enu_absolute_pos", 1000);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(timer_callback,_1, &tf_listener, &tf_buffer));

  ros::spin();

  return 0;
}
