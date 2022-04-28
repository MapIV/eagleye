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
 * rtk_deadreckoning_node.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static rtklib_msgs::RtklibNav _rtklib_nav;
static nmea_msgs::Gpgga _gga;
static geometry_msgs::Vector3Stamped _enu_vel;

static eagleye_msgs::Position _enu_absolute_rtk_deadreckoning;
static sensor_msgs::NavSatFix _eagleye_fix;
static eagleye_msgs::Heading _heading_interpolate_3rd;

static ros::Publisher _pub1;
static ros::Publisher _pub2;

struct RtkDeadreckoningParameter _rtk_deadreckoning_parameter;
struct RtkDeadreckoningStatus _rtk_deadreckoning_status;

static std::string _use_gnss_mode;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  _rtklib_nav = *msg;
}

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  _gga = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_3rd = *msg;
}


void timer_callback(const ros::TimerEvent& e, tf2_ros::TransformListener* tfListener_, tf2_ros::Buffer* tfBuffer_)
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_->lookupTransform(_rtk_deadreckoning_parameter.tf_gnss_parent_frame,
                                                  _rtk_deadreckoning_parameter.tf_gnss_child_frame, ros::Time(0));

    _rtk_deadreckoning_parameter.tf_gnss_translation_x = transformStamped.transform.translation.x;
    _rtk_deadreckoning_parameter.tf_gnss_translation_y = transformStamped.transform.translation.y;
    _rtk_deadreckoning_parameter.tf_gnss_translation_z = transformStamped.transform.translation.z;
    _rtk_deadreckoning_parameter.tf_gnss_rotation_x = transformStamped.transform.rotation.x;
    _rtk_deadreckoning_parameter.tf_gnss_rotation_y = transformStamped.transform.rotation.y;
    _rtk_deadreckoning_parameter.tf_gnss_rotation_z = transformStamped.transform.rotation.z;
    _rtk_deadreckoning_parameter.tf_gnss_rotation_w = transformStamped.transform.rotation.w;
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
  _enu_absolute_rtk_deadreckoning.header = msg->header;
  _enu_absolute_rtk_deadreckoning.header.frame_id = "base_link";
  _eagleye_fix.header = msg->header;
  _eagleye_fix.header.frame_id = "gnss";

  if (_use_gnss_mode == "rtklib" || _use_gnss_mode == "RTKLIB") // use RTKLIB mode
    rtk_deadreckoning_estimate(_rtklib_nav, _enu_vel, _gga, _heading_interpolate_3rd,
      _rtk_deadreckoning_parameter, &_rtk_deadreckoning_status, &_enu_absolute_rtk_deadreckoning, &_eagleye_fix);
  else if (_use_gnss_mode == "nmea" || _use_gnss_mode == "NMEA") // use NMEA mode
    rtk_deadreckoning_estimate(_enu_vel, _gga, _heading_interpolate_3rd,
      _rtk_deadreckoning_parameter, &_rtk_deadreckoning_status, &_enu_absolute_rtk_deadreckoning, &_eagleye_fix);    
  
  if(_enu_absolute_rtk_deadreckoning.status.enabled_status)
  {
    _pub1.publish(_enu_absolute_rtk_deadreckoning);
    _pub2.publish(_eagleye_fix);
  }
  else if (_gga.header.stamp.toSec() != 0)
  {
    sensor_msgs::NavSatFix fix;
    fix.header = _gga.header;
    fix.latitude = _gga.lat;
    fix.longitude = _gga.lon;
    fix.altitude = _gga.alt + _gga.undulation;
    _pub2.publish(fix);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtk_deadreckoning");
  ros::NodeHandle nh;

  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_gga_topic_name = "/navsat/gga";

  nh.getParam("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  nh.getParam("gga_topic",subscribe_gga_topic_name);
  nh.getParam("ecef_base_pos/x", _rtk_deadreckoning_parameter.ecef_base_pos_x);
  nh.getParam("ecef_base_pos/y", _rtk_deadreckoning_parameter.ecef_base_pos_y);
  nh.getParam("ecef_base_pos/z", _rtk_deadreckoning_parameter.ecef_base_pos_z);
  nh.getParam("ecef_base_pos/use_ecef_base_position", _rtk_deadreckoning_parameter.use_ecef_base_position);
  nh.getParam("rtk_deadreckoning/stop_judgment_velocity_threshold", _rtk_deadreckoning_parameter.stop_judgment_velocity_threshold);
  nh.getParam("tf_gnss_frame/parent", _rtk_deadreckoning_parameter.tf_gnss_parent_frame);
  nh.getParam("tf_gnss_frame/child", _rtk_deadreckoning_parameter.tf_gnss_child_frame);
  nh.getParam("use_gnss_mode",_use_gnss_mode);

  std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;
  std::cout<< "subscribe_gga_topic_name " << subscribe_gga_topic_name << std::endl;
  std::cout<< "ecef_base_pos_x " << _rtk_deadreckoning_parameter.ecef_base_pos_x << std::endl;
  std::cout<< "ecef_base_pos_y " << _rtk_deadreckoning_parameter.ecef_base_pos_y << std::endl;
  std::cout<< "ecef_base_pos_z " << _rtk_deadreckoning_parameter.ecef_base_pos_z << std::endl;
  std::cout<< "use_ecef_base_position " << _rtk_deadreckoning_parameter.use_ecef_base_position << std::endl;
  std::cout<< "stop_judgment_velocity_threshold " << _rtk_deadreckoning_parameter.stop_judgment_velocity_threshold << std::endl;
  std::cout<< "tf_gnss_frame/parent " << _rtk_deadreckoning_parameter.tf_gnss_parent_frame << std::endl;
  std::cout<< "tf_gnss_frame/child " << _rtk_deadreckoning_parameter.tf_gnss_child_frame << std::endl;
  std::cout<< "use_gnss_mode " << _use_gnss_mode << std::endl;

  ros::Subscriber sub1 = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  
  _pub1 = nh.advertise<eagleye_msgs::Position>("enu_absolute_rtk_deadreckoning", 1000);
  _pub2 = nh.advertise<sensor_msgs::NavSatFix>("rtk_fix", 1000);

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_(tfBuffer_);
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(timer_callback,_1, &tfListener_, &tfBuffer_));

  ros::spin();

  return 0;
}
