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
 * rtk_dead_reckoning_node.cpp
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

static eagleye_msgs::Position _enu_absolute_rtk_dead_reckoning;
static sensor_msgs::NavSatFix _eagleye_fix;
static eagleye_msgs::Heading _heading_interpolate_3rd;

static ros::Publisher _pub1;
static ros::Publisher _pub2;

struct RtkDeadreckoningParameter _rtk_dead_reckoning_parameter;
struct RtkDeadreckoningStatus _rtk_dead_reckoning_status;

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


void timer_callback(const ros::TimerEvent& e, tf2_ros::TransformListener* tf_listener, tf2_ros::Buffer* tf_buffer)
{
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer->lookupTransform(_rtk_dead_reckoning_parameter.tf_gnss_parent_frame,
                                                  _rtk_dead_reckoning_parameter.tf_gnss_child_frame, ros::Time(0));

    _rtk_dead_reckoning_parameter.tf_gnss_translation_x = transform_stamped.transform.translation.x;
    _rtk_dead_reckoning_parameter.tf_gnss_translation_y = transform_stamped.transform.translation.y;
    _rtk_dead_reckoning_parameter.tf_gnss_translation_z = transform_stamped.transform.translation.z;
    _rtk_dead_reckoning_parameter.tf_gnss_rotation_x = transform_stamped.transform.rotation.x;
    _rtk_dead_reckoning_parameter.tf_gnss_rotation_y = transform_stamped.transform.rotation.y;
    _rtk_dead_reckoning_parameter.tf_gnss_rotation_z = transform_stamped.transform.rotation.z;
    _rtk_dead_reckoning_parameter.tf_gnss_rotation_w = transform_stamped.transform.rotation.w;
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
  _enu_absolute_rtk_dead_reckoning.header = msg->header;
  _enu_absolute_rtk_dead_reckoning.header.frame_id = "base_link";
  _eagleye_fix.header = msg->header;
  _eagleye_fix.header.frame_id = "gnss";

  if (_use_gnss_mode == "rtklib" || _use_gnss_mode == "RTKLIB") // use RTKLIB mode
    rtk_dead_reckoning_estimate(_rtklib_nav, _enu_vel, _gga, _heading_interpolate_3rd,
      _rtk_dead_reckoning_parameter, &_rtk_dead_reckoning_status, &_enu_absolute_rtk_dead_reckoning, &_eagleye_fix);
  else if (_use_gnss_mode == "nmea" || _use_gnss_mode == "NMEA") // use NMEA mode
    rtk_dead_reckoning_estimate(_enu_vel, _gga, _heading_interpolate_3rd,
      _rtk_dead_reckoning_parameter, &_rtk_dead_reckoning_status, &_enu_absolute_rtk_dead_reckoning, &_eagleye_fix);

  if(_enu_absolute_rtk_dead_reckoning.status.enabled_status)
  {
    _pub1.publish(_enu_absolute_rtk_dead_reckoning);
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
  ros::init(argc, argv, "rtk_dead_reckoning");
  ros::NodeHandle nh;

  std::string subscribe_rtklib_nav_topic_name = "navsat/rtklib_nav";
  std::string subscribe_gga_topic_name = "navsat/gga";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_gnss_mode = conf["use_gnss_mode"].as<std::string>();

    _rtk_dead_reckoning_parameter.ecef_base_pos_x = conf["ecef_base_pos"]["x"].as<double>();
    _rtk_dead_reckoning_parameter.ecef_base_pos_y = conf["ecef_base_pos"]["y"].as<double>();
    _rtk_dead_reckoning_parameter.ecef_base_pos_z = conf["ecef_base_pos"]["z"].as<double>();
    _rtk_dead_reckoning_parameter.use_ecef_base_position = conf["ecef_base_pos"]["use_ecef_base_position"].as<bool>();
    _rtk_dead_reckoning_parameter.tf_gnss_parent_frame = conf["tf_gnss_frame"]["parent"].as<std::string>();
    _rtk_dead_reckoning_parameter.tf_gnss_child_frame = conf["tf_gnss_frame"]["child"].as<std::string>();
    _rtk_dead_reckoning_parameter.stop_judgement_threshold = conf["common"]["stop_judgement_threshold"].as<double>();

    _rtk_dead_reckoning_parameter.rtk_fix_STD = conf["rtk_dead_reckoning"]["rtk_fix_STD"].as<double>();
    _rtk_dead_reckoning_parameter.proc_noise = conf["rtk_dead_reckoning"]["proc_noise"].as<double>();

    std::cout << "use_gnss_mode " << _use_gnss_mode << std::endl;
    std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;

    std::cout << "ecef_base_pos_x " << _rtk_dead_reckoning_parameter.ecef_base_pos_x << std::endl;
    std::cout << "ecef_base_pos_y " << _rtk_dead_reckoning_parameter.ecef_base_pos_y << std::endl;
    std::cout << "ecef_base_pos_z " << _rtk_dead_reckoning_parameter.ecef_base_pos_z << std::endl;
    std::cout << "use_ecef_base_position " << _rtk_dead_reckoning_parameter.use_ecef_base_position << std::endl;
    std::cout << "tf_gnss_frame/parent " << _rtk_dead_reckoning_parameter.tf_gnss_parent_frame << std::endl;
    std::cout << "tf_gnss_frame/child " << _rtk_dead_reckoning_parameter.tf_gnss_child_frame << std::endl;
    std::cout << "stop_judgement_threshold " << _rtk_dead_reckoning_parameter.stop_judgement_threshold << std::endl;
    std::cout << "rtk_fix_STD " << _rtk_dead_reckoning_parameter.rtk_fix_STD << std::endl;
    std::cout << "proc_noise " << _rtk_dead_reckoning_parameter.proc_noise << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mrtk_dead_reckoning Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  ros::Subscriber sub1 = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());

  _pub1 = nh.advertise<eagleye_msgs::Position>("enu_absolute_pos_interpolate", 1000);
  _pub2 = nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(timer_callback,_1, &tf_listener, &tf_buffer));

  ros::spin();

  return 0;
}
