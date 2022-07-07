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

#include <eagleye_coordinate/eagleye_coordinate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtklib_msgs/msg/rtklib_nav.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

rclcpp::Publisher<rtklib_msgs::msg::RtklibNav>::SharedPtr _pub_rtk;
sensor_msgs::msg::NavSatFix::ConstSharedPtr _nav_msg_ptr;

void navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { _nav_msg_ptr = msg; }

void navpvt_callback(const ublox_msgs::msg::NavPVT::ConstSharedPtr msg)
{
  rtklib_msgs::msg::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp.sec = msg->sec;
  r.header.stamp.nanosec = msg->nano;
  if (_nav_msg_ptr != nullptr)
    r.status = *_nav_msg_ptr;
  r.tow = msg->i_tow;

  double llh[3];
  llh[0] = msg->lat * 1e-7 * M_PI / 180.0;  // [deg / 1e-7]->[rad]
  llh[1] = msg->lon * 1e-7 * M_PI / 180.0;  // [deg / 1e-7]->[rad]
  llh[2] = msg->height * 1e-3;              // [mm]->[m]
  double ecef_pos[3];
  llh2xyz(llh, ecef_pos);

  double enu_vel[3] = {msg->vel_e * 1e-3, msg->vel_n * 1e-3, -msg->vel_d * 1e-3};
  double ecef_vel[3];
  enu2xyz_vel(enu_vel, ecef_pos, ecef_vel);

  r.ecef_pos.x = ecef_pos[0];
  r.ecef_pos.y = ecef_pos[1];
  r.ecef_pos.z = ecef_pos[2];
  r.ecef_vel.x = ecef_vel[0];
  r.ecef_vel.y = ecef_vel[1];
  r.ecef_vel.z = ecef_vel[2];

  _pub_rtk->publish(r);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("navpvt2rtk");

  std::string ublox_navpvt_topic = "/sensing/gnss/ublox/navpvt";
  std::string nav_sat_fix_topic = "/sensing/gnss/ublox/nav_sat_fix";
  std::string rtklib_nav_topic = "/rtklib_nav";

  node->declare_parameter("rtklib_nav_topic", rtklib_nav_topic);
  node->declare_parameter("nav_sat_fix_topic", nav_sat_fix_topic);
  node->declare_parameter("ublox_navpvt_topic", ublox_navpvt_topic);

  node->get_parameter("rtklib_nav_topic", rtklib_nav_topic);
  node->get_parameter("ublox_navpvt_topic", ublox_navpvt_topic);
  node->get_parameter("nav_sat_fix_topic", nav_sat_fix_topic);

  std::cout << "rtklib_nav_topic: " << rtklib_nav_topic << std::endl;
  std::cout << "ublox_navpvt_topic: " << ublox_navpvt_topic << std::endl;
  std::cout << "nav_sat_fix_topic: " << nav_sat_fix_topic << std::endl;


  auto sub_ublox = node->create_subscription<ublox_msgs::msg::NavPVT>(ublox_navpvt_topic, 1000, navpvt_callback);
  auto sub_fix = node->create_subscription<sensor_msgs::msg::NavSatFix>(nav_sat_fix_topic, 1000, navsatfix_callback);
  _pub_rtk = node->create_publisher<rtklib_msgs::msg::RtklibNav>(rtklib_nav_topic, 10);

  rclcpp::spin(node);

  return 0;
}
