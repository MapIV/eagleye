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

#include "rclcpp/rclcpp.hpp"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::msg::Imu imu;
static rtklib_msgs::msg::RtklibNav rtklib_nav;
//static sensor_msgs::msg::NavSatFix fix;
static sensor_msgs::msg::NavSatFix f9p_fix;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
//static eagleye_msgs::msg::Distance distance;
//static eagleye_msgs::msg::Heading heading_1st;
//static eagleye_msgs::msg::Heading heading_interpolate_1st;
//static eagleye_msgs::msg::Heading heading_2nd;
//static eagleye_msgs::msg::Heading heading_interpolate_2nd;
//static eagleye_msgs::msg::Heading heading_3rd;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
//static eagleye_msgs::msg::YawrateOffset yawrate_offset_1st;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::msg::SlipAngle slip_angle;
static eagleye_msgs::msg::Height height;
static eagleye_msgs::msg::Pitching pitching;
//static eagleye_msgs::msg::Position enu_relative_pos;
//static geometry_msgs::msg::Vector3Stamped enu_vel;
//static eagleye_msgs::msg::Position enu_absolute_pos;
static eagleye_msgs::msg::Position enu_absolute_pos_interpolate;
static sensor_msgs::msg::NavSatFix eagleye_fix;
//static geometry_msgs::msg::TwistStamped eagleye_twist;

static bool f9p_fix_sub_status;

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

/*
void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
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

void f9p_fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
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

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity.header = msg->header;
  velocity.twist = msg->twist;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

/*
void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}
*/

/*
void heading_1st_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_1st.header = msg->header;
  heading_1st.heading_angle = msg->heading_angle;
  heading_1st.status = msg->status;
}
*/

/*
void heading_interpolate_1st_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_1st.header = msg->header;
  heading_interpolate_1st.heading_angle = msg->heading_angle;
  heading_interpolate_1st.status = msg->status;
}
*/

/*
void heading_2nd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_2nd.header = msg->header;
  heading_2nd.heading_angle = msg->heading_angle;
  heading_2nd.status = msg->status;
}
*/

/*
void heading_interpolate_2nd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_2nd.header = msg->header;
  heading_interpolate_2nd.heading_angle = msg->heading_angle;
  heading_interpolate_2nd.status = msg->status;
}
*/

/*
void heading_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_3rd.header = msg->header;
  heading_3rd.heading_angle = msg->heading_angle;
  heading_3rd.status = msg->status;
}
*/

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

/*
void yawrate_offset_1st_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_1st.header = msg->header;
  yawrate_offset_1st.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_1st.status = msg->status;
}
*/

void yawrate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle.header = msg->header;
  slip_angle.coefficient = msg->coefficient;
  slip_angle.slip_angle = msg->slip_angle;
  slip_angle.status = msg->status;
}

/*
void enu_relative_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_relative_pos.header = msg->header;
  enu_relative_pos.enu_pos = msg->enu_pos;
  enu_relative_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_relative_pos.status = msg->status;
}
*/

/*
void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
}
*/

/*
void enu_absolute_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}
*/

void height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
{
  height.header = msg->header;
  height.height = msg->height;
  height.status = msg->status;
}

void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
{
  pitching.header = msg->header;
  pitching.pitching_angle = msg->pitching_angle;
  pitching.status = msg->status;
}


void enu_absolute_pos_interpolate_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_absolute_pos_interpolate.header = msg->header;
  enu_absolute_pos_interpolate.enu_pos = msg->enu_pos;
  enu_absolute_pos_interpolate.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos_interpolate.status = msg->status;
}

void eagleye_fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
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
void eagleye_twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  eagleye_twist.header = msg->header;
  eagleye_twist.twist = msg->twist;
}
*/

// void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
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
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("monitor_node");

  std::string subscribe_twist_topic_name = "can_twist";
  std::string subscribe_imu_topic_name = "imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "rtklib_nav";
  std::string subscribe_navsatfix_topic_name = "f9p/fix";

  node->declare_parameter("twist_topic" , "can_twist");
  node->declare_parameter("imu_topic" , "imu/data_raw");
  node->declare_parameter("rtklib_nav_topic" , "rtklib_nav");
  node->declare_parameter("navsatfix_topic" , "f9p/fix");

  subscribe_twist_topic_name = node->get_parameter("twist_topic").as_string();
  subscribe_imu_topic_name = node->get_parameter("imu_topic").as_string();
  subscribe_rtklib_nav_topic_name = node->get_parameter("rtklib_nav_topic").as_string();
  subscribe_navsatfix_topic_name = node->get_parameter("navsatfix_topic").as_string();

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback); //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10), fix_callback); //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_navsatfix_topic_name, 1000, f9p_fix_callback); //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, velocity_callback); //ros::TransportHints().tcpNoDelay()
  auto sub6 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("/eagleye/velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub7 = node->create_subscription<eagleye_msgs::msg::Distance>("/eagleye/distance", rclcpp::QoS(10), distance_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub8 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_1st", rclcpp::QoS(10), heading_1st_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub9 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_interpolate_1st", rclcpp::QoS(10), heading_interpolate_1st_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub10 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_2nd", rclcpp::QoS(10), heading_2nd_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub11 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_interpolate_2nd", rclcpp::QoS(10), heading_interpolate_2nd_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub12 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_3rd", rclcpp::QoS(10), heading_3rd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub13 = node->create_subscription<eagleye_msgs::msg::Heading>("/eagleye/heading_interpolate_3rd", rclcpp::QoS(10), heading_interpolate_3rd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub14 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("/eagleye/yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub15 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("/eagleye/yawrate_offset_1st", rclcpp::QoS(10), yawrate_offset_1st_callback); //ros::TransportHints().tcpNoDelay()
  auto sub16 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("/eagleye/yawrate_offset_2nd", rclcpp::QoS(10), yawrate_offset_2nd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub17 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("/eagleye/slip_angle", rclcpp::QoS(10), slip_angle_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub18 = node->create_subscription<eagleye_msgs::msg::Position>("/eagleye/enu_relative_pos", rclcpp::QoS(10), enu_relative_pos_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub19 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("/eagleye/enu_vel", rclcpp::QoS(10), enu_vel_callback); //ros::TransportHints().tcpNoDelay()
  auto sub20 = node->create_subscription<eagleye_msgs::msg::Height>("/eagleye/height", rclcpp::QoS(10), height_callback); //ros::TransportHints().tcpNoDelay()
  auto sub21 = node->create_subscription<eagleye_msgs::msg::Pitching>("/eagleye/pitching", rclcpp::QoS(10), pitching_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub22 = node->create_subscription<eagleye_msgs::msg::Position>("/eagleye/enu_absolute_pos", rclcpp::QoS(10), enu_absolute_pos_callback); //ros::TransportHints().tcpNoDelay()
  auto sub23 = node->create_subscription<eagleye_msgs::msg::Position>("/eagleye/enu_absolute_pos_interpolate", rclcpp::QoS(10), enu_absolute_pos_interpolate_callback); //ros::TransportHints().tcpNoDelay()
  auto sub24 = node->create_subscription<sensor_msgs::msg::NavSatFix>("/eagleye/fix", rclcpp::QoS(10), eagleye_fix_callback); //ros::TransportHints().tcpNoDelay()
  // auto sub25 = node->create_subscription<geometry_msgs::msg::TwistStamped>("/eagleye/twist", rclcpp::QoS(10), eagleye_twist_callback); //ros::TransportHints().tcpNoDelay()

  rclcpp::spin(node);

  return 0;
}
