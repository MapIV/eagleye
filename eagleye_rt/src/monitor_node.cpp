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
 * monitor.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::Imu imu;
static rtklib_msgs::RtklibNav rtklib_nav;
//static sensor_msgs::NavSatFix fix;
static sensor_msgs::NavSatFix f9p_fix;
static geometry_msgs::TwistStamped velocity;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
//static eagleye_msgs::Distance distance;
//static eagleye_msgs::Heading heading_1st;
//static eagleye_msgs::Heading heading_interpolate_1st;
//static eagleye_msgs::Heading heading_2nd;
//static eagleye_msgs::Heading heading_interpolate_2nd;
//static eagleye_msgs::Heading heading_3rd;
static eagleye_msgs::Heading heading_interpolate_3rd;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
//static eagleye_msgs::YawrateOffset yawrate_offset_1st;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::SlipAngle slip_angle;
static eagleye_msgs::Height height;
static eagleye_msgs::Pitching pitching;
//static eagleye_msgs::Position enu_relative_pos;
//static geometry_msgs::Vector3Stamped enu_vel;
//static eagleye_msgs::Position enu_absolute_pos;
static eagleye_msgs::Position enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix eagleye_fix;
//static geometry_msgs::TwistStamped eagleye_twist;

static bool f9p_fix_sub_status;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

/*
void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  fix.header = msg->header;
  fix.status = msg->status;
  fix.latitude = msg->latitude;
  fix.longitude = msg->longitude;
  fix.altitude = msg->altitude;
  fix.position_covariance = msg->position_covariance;
  fix.position_covariance_type = msg->position_covariance_type;
}
*/

void f9p_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  f9p_fix.header = msg->header;
  f9p_fix.status = msg->status;
  f9p_fix.latitude = msg->latitude;
  f9p_fix.longitude = msg->longitude;
  f9p_fix.altitude = msg->altitude;
  f9p_fix.position_covariance = msg->position_covariance;
  f9p_fix.position_covariance_type = msg->position_covariance_type;
  f9p_fix_sub_status = true;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  velocity.header = msg->header;
  velocity.twist = msg->twist;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

/*
void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}
*/

/*
void heading_1st_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_1st.header = msg->header;
  heading_1st.heading_angle = msg->heading_angle;
  heading_1st.status = msg->status;
}
*/

/*
void heading_interpolate_1st_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_1st.header = msg->header;
  heading_interpolate_1st.heading_angle = msg->heading_angle;
  heading_interpolate_1st.status = msg->status;
}
*/

/*
void heading_2nd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_2nd.header = msg->header;
  heading_2nd.heading_angle = msg->heading_angle;
  heading_2nd.status = msg->status;
}
*/

/*
void heading_interpolate_2nd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_2nd.header = msg->header;
  heading_interpolate_2nd.heading_angle = msg->heading_angle;
  heading_interpolate_2nd.status = msg->status;
}
*/

/*
void heading_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_3rd.header = msg->header;
  heading_3rd.heading_angle = msg->heading_angle;
  heading_3rd.status = msg->status;
}
*/

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

/*
void yawrate_offset_1st_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_1st.header = msg->header;
  yawrate_offset_1st.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_1st.status = msg->status;
}
*/

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void slip_angle_callback(const eagleye_msgs::SlipAngle::ConstPtr& msg)
{
  slip_angle.header = msg->header;
  slip_angle.coefficient = msg->coefficient;
  slip_angle.slip_angle = msg->slip_angle;
  slip_angle.status = msg->status;
}

/*
void enu_relative_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_relative_pos.header = msg->header;
  enu_relative_pos.enu_pos = msg->enu_pos;
  enu_relative_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_relative_pos.status = msg->status;
}
*/

/*
void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
}
*/

/*
void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}
*/

void height_callback(const eagleye_msgs::Height::ConstPtr& msg)
{
  height.header = msg->header;
  height.height = msg->height;
  height.status = msg->status;
}

void pitching_callback(const eagleye_msgs::Pitching::ConstPtr& msg)
{
  pitching.header = msg->header;
  pitching.pitching_angle = msg->pitching_angle;
  pitching.status = msg->status;
}


void enu_absolute_pos_interpolate_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos_interpolate.header = msg->header;
  enu_absolute_pos_interpolate.enu_pos = msg->enu_pos;
  enu_absolute_pos_interpolate.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos_interpolate.status = msg->status;
}

void eagleye_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  eagleye_fix.header = msg->header;
  eagleye_fix.status = msg->status;
  eagleye_fix.latitude = msg->latitude;
  eagleye_fix.longitude = msg->longitude;
  eagleye_fix.altitude = msg->altitude;
  eagleye_fix.position_covariance = msg->position_covariance;
  eagleye_fix.position_covariance_type = msg->position_covariance_type;
}


/*
void eagleye_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  eagleye_twist.header = msg->header;
  eagleye_twist.twist = msg->twist;
}
*/

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  std::cout << std::endl;
  std::cout<<"\033[1;33m Eagleye status \033[m"<<std::endl;
  std::cout << std::endl;
  std::cout << std::fixed;

  std::cout << "--- \033[1;34m imu(input)\033[m ------------------------------"<< std::endl;
  std::cout<<"\033[1m linear_acceleration \033[mx "<<std::setprecision(6)<<imu.linear_acceleration.x<<" [m/s^2]"<<std::endl;
  std::cout<<"\033[1m linear acceleration \033[my "<<std::setprecision(6)<<imu.linear_acceleration.y<<" [m/s^2]"<<std::endl;
  std::cout<<"\033[1m linear acceleration \033[mz "<<std::setprecision(6)<<imu.linear_acceleration.z<<" [m/s^2]"<<std::endl;
  std::cout<<"\033[1m angular velocity \033[mx "<<std::setprecision(6)<<imu.angular_velocity.x<<" [rad/s]"<<std::endl;
  std::cout<<"\033[1m angular velocity \033[my "<<std::setprecision(6)<<imu.angular_velocity.y<<" [rad/s]"<<std::endl;
  std::cout<<"\033[1m angular velocity \033[mz "<<std::setprecision(6)<<imu.angular_velocity.z<<" [rad/s]"<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m velocity(input)\033[m -------------------------"<< std::endl;
  std::cout<<"\033[1m velocity \033[m"<<std::setprecision(4)<<velocity.twist.linear.x * 3.6<<" [km/h]"<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m rtklib(input)\033[m ---------------------------"<< std::endl;
  std::cout<<"\033[1m time of week  \033[m"<<rtklib_nav.tow<<" [ms]"<<std::endl;
  std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<rtklib_nav.status.latitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<rtklib_nav.status.longitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<rtklib_nav.status.altitude<<" [m]"<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m f9p(input)\033[m ------------------------------"<< std::endl;

  if(f9p_fix_sub_status)
  {
    std::cout<< "\033[1m rtk status \033[m "<<int(f9p_fix.status.status)<<std::endl;
    std::cout<< "\033[1m rtk status \033[m "<<(f9p_fix.status.status ? "\033[1;31mNo Fix\033[m" : "\033[1;32mFix\033[m")<<std::endl;
    std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<f9p_fix.latitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<f9p_fix.longitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<f9p_fix.altitude<<" [m]"<<std::endl;
    std::cout << std::endl;
  }
  else
  {
    std::cout << std::endl;
    std::cout<<"\033[1;31m no subscription \033[m"<<std::endl;
    std::cout << std::endl;
  }


  std::cout << "--- \033[1;34m velocity SF\033[m -----------------------------"<< std::endl;
  std::cout<<"\033[1m scale factor \033[m "<<std::setprecision(4)<<velocity_scale_factor.scale_factor<<std::endl;
  std::cout<<"\033[1m correction velocity \033[m "<<std::setprecision(4)<<velocity_scale_factor.correction_velocity.linear.x * 3.6<<" [km/h]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(velocity_scale_factor.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m yawrate offset stop\033[m ---------------------"<< std::endl;
  std::cout<<"\033[1m yawrate offset \033[m "<<std::setprecision(6)<<yawrate_offset_stop.yawrate_offset<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(yawrate_offset_stop.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m yawrate offset\033[m --------------------------"<< std::endl;
  std::cout<<"\033[1m yawrate offset \033[m "<<std::setprecision(6)<<yawrate_offset_2nd.yawrate_offset<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(yawrate_offset_2nd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m slip angle\033[m ------------------------------"<< std::endl;
  std::cout<<"\033[1m coefficient \033[m "<<std::setprecision(6)<<slip_angle.coefficient<<std::endl;
  std::cout<<"\033[1m slip angle \033[m "<<std::setprecision(6)<<slip_angle.slip_angle<<" [rad]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(slip_angle.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m heading\033[m ---------------------------------"<< std::endl;
  std::cout<<"\033[1m heading \033[m "<<std::setprecision(6)<<heading_interpolate_3rd.heading_angle<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(heading_interpolate_3rd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m pitching\033[m --------------------------------"<< std::endl;
  std::cout<<"\033[1m pitching \033[m "<<std::setprecision(6)<<pitching.pitching_angle<<" [rad]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(pitching.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m height\033[m ----------------------------------"<< std::endl;
  std::cout<<"\033[1m height \033[m "<<std::setprecision(4)<<height.height<<" [m]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(height.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m position\033[m --------------------------------"<< std::endl;
  std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<eagleye_fix.latitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<eagleye_fix.longitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<eagleye_fix.altitude<<" [m]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(enu_absolute_pos_interpolate.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "monitor");

  ros::NodeHandle n;

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_navsatfix_topic_name = "/f9p/fix";

  n.getParam("eagleye/twist_topic",subscribe_twist_topic_name);
  n.getParam("eagleye/imu_topic",subscribe_imu_topic_name);
  n.getParam("eagleye/rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("eagleye/navsatfix_topic",subscribe_navsatfix_topic_name);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;

  ros::Subscriber sub1 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub3 = n.subscribe("fix", 1000, fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_navsatfix_topic_name, 1000, f9p_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub7 = n.subscribe("eagleye/distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub8 = n.subscribe("eagleye/heading_1st", 1000, heading_1st_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub9 = n.subscribe("eagleye/heading_interpolate_1st", 1000, heading_interpolate_1st_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub10 = n.subscribe("eagleye/heading_2nd", 1000, heading_2nd_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub11 = n.subscribe("eagleye/heading_interpolate_2nd", 1000, heading_interpolate_2nd_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub12 = n.subscribe("eagleye/heading_3rd", 1000, heading_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub13 = n.subscribe("eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub14 = n.subscribe("eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub15 = n.subscribe("eagleye/yawrate_offset_1st", 1000, yawrate_offset_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub16 = n.subscribe("eagleye/yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub17 = n.subscribe("eagleye/slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub18 = n.subscribe("eagleye/enu_relative_pos", 1000, enu_relative_pos_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub19 = n.subscribe("eagleye/enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub20 = n.subscribe("eagleye/height", 1000, height_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub21 = n.subscribe("eagleye/pitching", 1000, pitching_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub22 = n.subscribe("eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub23 = n.subscribe("eagleye/enu_absolute_pos_interpolate", 1000, enu_absolute_pos_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub24 = n.subscribe("eagleye/fix", 1000, eagleye_fix_callback, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber sub25 = n.subscribe("eagleye/twist", 1000, eagleye_twist_callback, ros::TransportHints().tcpNoDelay());

  ros::spin();

  return 0;
}
