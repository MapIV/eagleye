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
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

static sensor_msgs::msg::Imu imu;
static rtklib_msgs::msg::RtklibNav rtklib_nav;
static sensor_msgs::msg::NavSatFix fix;
static sensor_msgs::msg::NavSatFix navsat_fix;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Distance distance;
static eagleye_msgs::msg::Heading heading_1st;
static eagleye_msgs::msg::Heading heading_interpolate_1st;
static eagleye_msgs::msg::Heading heading_2nd;
static eagleye_msgs::msg::Heading heading_interpolate_2nd;
static eagleye_msgs::msg::Heading heading_3rd;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_1st;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::msg::SlipAngle slip_angle;
static eagleye_msgs::msg::Height height;
static eagleye_msgs::msg::Pitching pitching;
static eagleye_msgs::msg::Position enu_relative_pos;
static geometry_msgs::msg::Vector3Stamped enu_vel;
static eagleye_msgs::msg::Position enu_absolute_pos;
static eagleye_msgs::msg::Position enu_absolute_pos_interpolate;
static sensor_msgs::msg::NavSatFix eagleye_fix;
static geometry_msgs::msg::TwistStamped eagleye_twist;

static bool navsat_fix_sub_status;
static bool print_status;

static double imu_time_last;
static double rtklib_nav_time_last;
static double navsat_fix_time_last;
static double velocity_time_last;
static double velocity_scale_factor_time_last;
static double distance_time_last;
static double heading_1st_time_last;
static double heading_interpolate_1st_time_last;
static double heading_2nd_time_last;
static double heading_interpolate_2nd_time_last;
static double heading_3rd_time_last;
static double heading_interpolate_3rd_time_last;
static double yawrate_offset_stop_time_last;
static double yawrate_offset_1st_time_last;
static double yawrate_offset_2nd_time_last;
static double slip_angle_time_last;
static double height_time_last;
static double pitching_time_last;
static double enu_vel_time_last;
static double enu_absolute_pos_time_last;
static double enu_absolute_pos_interpolate_time_last;
static double eagleye_twist_time_last;

static double update_rate = 10;
static double th_gnss_deadrock_time = 10;

std::shared_ptr<diagnostic_updater::Updater> updater_;

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  fix = *msg;
}

void navsatfix_fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  navsat_fix = *msg;
  navsat_fix_sub_status = true;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance = *msg;
}

void heading_1st_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_1st = *msg;
}

void heading_interpolate_1st_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_1st = *msg;
}

void heading_2nd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_2nd = *msg;
}

void heading_interpolate_2nd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_2nd = *msg;
}

void heading_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_3rd = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop = *msg;
}

void yawrate_offset_1st_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_1st = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_2nd = *msg;
}

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle = *msg;
}

void enu_relative_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_relative_pos = *msg;
}

void enu_vel_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
{
  enu_vel = *msg;
}

void enu_absolute_pos_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_absolute_pos = *msg;
}

void height_callback(const eagleye_msgs::msg::Height::ConstSharedPtr msg)
{
  height = *msg;
}

void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
{
  pitching = *msg;
}


void enu_absolute_pos_interpolate_callback(const eagleye_msgs::msg::Position::ConstSharedPtr msg)
{
  enu_absolute_pos_interpolate = *msg;
}

void eagleye_fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  eagleye_fix = *msg;
}

void eagleye_twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  eagleye_twist = *msg;
}

void imu_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(imu.header.stamp);
  auto imu_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (imu_time_last == imu_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }

  imu_time_last = imu_time;
  stat.summary(level, msg);
}
void rtklib_nav_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(rtklib_nav.header.stamp);
  auto rtklib_nav_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (rtklib_nav_time_last - rtklib_nav_time > th_gnss_deadrock_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }

  rtklib_nav_time_last = rtklib_nav_time;
  stat.summary(level, msg);
}
void navsat_fix_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(navsat_fix.header.stamp);
  auto navsat_fix_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (navsat_fix_time_last - navsat_fix_time > th_gnss_deadrock_time || !navsat_fix_sub_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  navsat_fix_time_last = navsat_fix_time;
  stat.summary(level, msg);
}
void velocity_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(velocity.header.stamp);
  auto velocity_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (velocity_time_last == velocity_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }

  velocity_time_last = velocity_time;
  stat.summary(level, msg);
}
void velocity_scale_factor_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(velocity_scale_factor.header.stamp);
  auto velocity_scale_factor_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (velocity_scale_factor_time_last == velocity_scale_factor_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(velocity_scale_factor.scale_factor)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!velocity_scale_factor.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  velocity_scale_factor_time_last = velocity_scale_factor_time;
  stat.summary(level, msg);
}
void distance_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(distance.header.stamp);
  auto distance_time = ros_clock.seconds();
  
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (distance_time_last == distance_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(distance.distance)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!distance.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  distance_time_last = distance_time;
  stat.summary(level, msg);
}
void heading_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(heading_1st.header.stamp);
  auto heading_1st_time = ros_clock.seconds();
  
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(heading_1st.heading_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (heading_1st_time_last - heading_1st_time > th_gnss_deadrock_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!heading_1st.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_1st_time_last = heading_1st_time;
  stat.summary(level, msg);
}
void heading_interpolate_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(heading_interpolate_1st.header.stamp);
  auto heading_interpolate_1st_time = ros_clock.seconds();
  
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (heading_interpolate_1st_time_last == heading_interpolate_1st_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(heading_interpolate_1st.heading_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!heading_interpolate_1st.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_interpolate_1st_time_last = heading_interpolate_1st_time;
  stat.summary(level, msg);
}
void heading_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(heading_2nd.header.stamp);
  auto heading_2nd_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(heading_2nd.heading_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (heading_2nd_time_last - heading_2nd_time > th_gnss_deadrock_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!heading_2nd.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_2nd_time_last = heading_2nd_time;
  stat.summary(level, msg);
}
void heading_interpolate_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(heading_interpolate_2nd.header.stamp);
  auto heading_interpolate_2nd_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (heading_interpolate_2nd_time_last == heading_interpolate_2nd_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(heading_interpolate_2nd.heading_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!heading_interpolate_2nd.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_interpolate_2nd_time_last = heading_interpolate_2nd_time;
  stat.summary(level, msg);
}
void heading_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(heading_3rd.header.stamp);
  auto heading_3rd_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(heading_3rd.heading_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (heading_3rd_time_last - heading_3rd_time > th_gnss_deadrock_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!heading_3rd.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_3rd_time_last = heading_3rd_time;
  stat.summary(level, msg);
}
void heading_interpolate_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(heading_interpolate_3rd.header.stamp);
  auto heading_interpolate_3rd_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (heading_interpolate_3rd_time_last == heading_interpolate_3rd_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(heading_interpolate_3rd.heading_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!heading_interpolate_3rd.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_interpolate_3rd_time_last = heading_interpolate_3rd_time;
  stat.summary(level, msg);
}
void yawrate_offset_stop_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(yawrate_offset_stop.header.stamp);
  auto yawrate_offset_stop_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (yawrate_offset_stop_time_last == yawrate_offset_stop_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(yawrate_offset_stop.yawrate_offset)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!yawrate_offset_stop.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  yawrate_offset_stop_time_last = yawrate_offset_stop_time;
  stat.summary(level, msg);
}
void yawrate_offset_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(yawrate_offset_1st.header.stamp);
  auto yawrate_offset_1st_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (yawrate_offset_1st_time_last == yawrate_offset_1st_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(yawrate_offset_1st.yawrate_offset)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!yawrate_offset_1st.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  yawrate_offset_1st_time_last = yawrate_offset_1st_time;
  stat.summary(level, msg);
}
void yawrate_offset_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(yawrate_offset_2nd.header.stamp);
  auto yawrate_offset_2nd_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (yawrate_offset_2nd_time_last == yawrate_offset_2nd_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(yawrate_offset_2nd.yawrate_offset)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!yawrate_offset_2nd.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  yawrate_offset_2nd_time_last = yawrate_offset_2nd_time;
  stat.summary(level, msg);
}
void slip_angle_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(slip_angle.header.stamp);
  auto slip_angle_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (slip_angle_time_last == slip_angle_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(slip_angle.slip_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (slip_angle.coefficient == 0) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "/slip_angle/manual_coefficient is not set";
  }
  else if (!slip_angle.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  slip_angle_time_last = slip_angle_time;
  stat.summary(level, msg);
}
void enu_vel_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(enu_vel.header.stamp);
  auto enu_vel_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

 if (!std::isfinite(enu_vel.vector.x)||!std::isfinite(enu_vel.vector.y)||!std::isfinite(enu_vel.vector.z)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else  if (enu_vel_time_last == enu_vel_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  enu_vel_time_last = enu_vel_time;
  stat.summary(level, msg);
}
void height_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(height.header.stamp);
  auto height_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (height_time_last == height_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(height.height)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!height.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  height_time_last = height_time;
  stat.summary(level, msg);
}
void pitching_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(pitching.header.stamp);
  auto pitching_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (pitching_time_last == pitching_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(pitching.pitching_angle)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!pitching.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  pitching_time_last = pitching_time;
  stat.summary(level, msg);
}
void enu_absolute_pos_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(enu_absolute_pos.header.stamp);
  auto enu_absolute_pos_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(enu_absolute_pos.enu_pos.x)||!std::isfinite(enu_absolute_pos.enu_pos.y)||!std::isfinite(enu_absolute_pos.enu_pos.z)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (enu_absolute_pos_time_last - enu_absolute_pos_time > th_gnss_deadrock_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!enu_absolute_pos.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  enu_absolute_pos_time_last = enu_absolute_pos_time;
  stat.summary(level, msg);
}
void enu_absolute_pos_interpolate_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(enu_absolute_pos_interpolate.header.stamp);
  auto enu_absolute_pos_interpolate_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(enu_absolute_pos_interpolate.enu_pos.x)||!std::isfinite(enu_absolute_pos_interpolate.enu_pos.y)||!std::isfinite(enu_absolute_pos_interpolate.enu_pos.z)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (enu_absolute_pos_interpolate_time_last == enu_absolute_pos_interpolate_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!enu_absolute_pos_interpolate.status.enabled_status) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  enu_absolute_pos_interpolate_time_last = enu_absolute_pos_interpolate_time;
  stat.summary(level, msg);
}
void twist_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp::Time ros_clock(eagleye_twist.header.stamp);
  auto eagleye_twist_time = ros_clock.seconds();

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (eagleye_twist_time_last == eagleye_twist_time) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!std::isfinite(eagleye_twist.twist.linear.x)||!std::isfinite(eagleye_twist.twist.linear.y)||!std::isfinite(eagleye_twist.twist.linear.z)
      ||!std::isfinite(eagleye_twist.twist.angular.x)||!std::isfinite(eagleye_twist.twist.angular.y)||!std::isfinite(eagleye_twist.twist.angular.z)) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }

  eagleye_twist_time_last = eagleye_twist_time;
  stat.summary(level, msg);
}

void printStatus(void)
{
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

  std::cout << "--- \033[1;34m navsat(input)\033[m ------------------------------"<< std::endl;

  if (navsat_fix_sub_status)
  {
    std::cout<< "\033[1m rtk status \033[m "<<int(navsat_fix.status.status)<<std::endl;
    std::cout<< "\033[1m rtk status \033[m "<<(navsat_fix.status.status ? "\033[1;31mNo Fix\033[m" : "\033[1;32mFix\033[m")<<std::endl;
    std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<navsat_fix.latitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<navsat_fix.longitude<<" [deg]"<<std::endl;
    std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<navsat_fix.altitude<<" [m]"<<std::endl;
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

void on_timer()
{
  // Diagnostic Updater
  updater_->force_update();

}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  if (print_status)
  {
    printStatus();
  }

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("monitor");
  updater_ = std::make_shared<diagnostic_updater::Updater>(node);

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_navsatfix_topic_name = "/navsatfix/fix";

  node->declare_parameter("twist_topic",subscribe_twist_topic_name);
  node->declare_parameter("imu_topic",subscribe_imu_topic_name);
  node->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->declare_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->declare_parameter("monitor.print_status",print_status);

  node->get_parameter("twist_topic",subscribe_twist_topic_name);
  node->get_parameter("imu_topic",subscribe_imu_topic_name);
  node->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->get_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->get_parameter("monitor.print_status",print_status);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;
  std::cout<< "print_status "<<print_status<<std::endl;

  // // Diagnostic Updater
  updater_->setHardwareID("topic_checker");
  updater_->add("eagleye_input_imu", imu_topic_checker);
  updater_->add("eagleye_input_rtklib_nav", rtklib_nav_topic_checker);
  updater_->add("eagleye_input_navsat_fix", navsat_fix_topic_checker);
  updater_->add("eagleye_input_velocity", velocity_topic_checker);
  updater_->add("eagleye_velocity_scale_factor", velocity_scale_factor_topic_checker);
  updater_->add("eagleye_distance", distance_topic_checker);
  updater_->add("eagleye_heading_1st", heading_1st_topic_checker);
  updater_->add("eagleye_heading_interpolate_1st", heading_interpolate_1st_topic_checker);
  updater_->add("eagleye_heading_2nd", heading_2nd_topic_checker);
  updater_->add("eagleye_heading_interpolate_2nd", heading_interpolate_2nd_topic_checker);
  updater_->add("eagleye_heading_3rd", heading_3rd_topic_checker);
  updater_->add("eagleye_heading_interpolate_3rd", heading_interpolate_3rd_topic_checker);
  updater_->add("eagleye_yawrate_offset_stop", yawrate_offset_stop_topic_checker);
  updater_->add("eagleye_yawrate_offset_1st", yawrate_offset_1st_topic_checker);
  updater_->add("eagleye_yawrate_offset_2nd", yawrate_offset_2nd_topic_checker);
  updater_->add("eagleye_slip_angle", slip_angle_topic_checker);
  updater_->add("eagleye_enu_vel", enu_vel_topic_checker);
  updater_->add("eagleye_height", height_topic_checker);
  updater_->add("eagleye_pitching", pitching_topic_checker);
  updater_->add("eagleye_enu_absolute_pos", enu_absolute_pos_topic_checker);
  updater_->add("eagleye_enu_absolute_pos_interpolate", enu_absolute_pos_interpolate_topic_checker);
  updater_->add("eagleye_twist", twist_topic_checker);

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback); //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback); //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10), fix_callback); //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_navsatfix_topic_name, 1000, navsatfix_fix_callback); //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, velocity_callback); //ros::TransportHints().tcpNoDelay()
  auto sub6 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback); //ros::TransportHints().tcpNoDelay()
  auto sub7 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", rclcpp::QoS(10), distance_callback); //ros::TransportHints().tcpNoDelay()
  auto sub8 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_1st", rclcpp::QoS(10), heading_1st_callback); //ros::TransportHints().tcpNoDelay()
  auto sub9 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_1st", rclcpp::QoS(10), heading_interpolate_1st_callback); //ros::TransportHints().tcpNoDelay()
  auto sub10 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_2nd", rclcpp::QoS(10), heading_2nd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub11 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_2nd", rclcpp::QoS(10), heading_interpolate_2nd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub12 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_3rd", rclcpp::QoS(10), heading_3rd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub13 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", rclcpp::QoS(10), heading_interpolate_3rd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub14 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback); //ros::TransportHints().tcpNoDelay()
  auto sub15 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_1st", rclcpp::QoS(10), yawrate_offset_1st_callback); //ros::TransportHints().tcpNoDelay()
  auto sub16 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_2nd", rclcpp::QoS(10), yawrate_offset_2nd_callback); //ros::TransportHints().tcpNoDelay()
  auto sub17 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10), slip_angle_callback); //ros::TransportHints().tcpNoDelay()
  auto sub18 = node->create_subscription<eagleye_msgs::msg::Position>("enu_relative_pos", rclcpp::QoS(10), enu_relative_pos_callback); //ros::TransportHints().tcpNoDelay()
  auto sub19 = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", rclcpp::QoS(10), enu_vel_callback); //ros::TransportHints().tcpNoDelay()
  auto sub20 = node->create_subscription<eagleye_msgs::msg::Height>("height", rclcpp::QoS(10), height_callback); //ros::TransportHints().tcpNoDelay()
  auto sub21 = node->create_subscription<eagleye_msgs::msg::Pitching>("pitching", rclcpp::QoS(10), pitching_callback); //ros::TransportHints().tcpNoDelay()
  auto sub22 = node->create_subscription<eagleye_msgs::msg::Position>("enu_absolute_pos", rclcpp::QoS(10), enu_absolute_pos_callback); //ros::TransportHints().tcpNoDelay()
  auto sub23 = node->create_subscription<eagleye_msgs::msg::Position>("enu_absolute_pos_interpolate", rclcpp::QoS(10), enu_absolute_pos_interpolate_callback); //ros::TransportHints().tcpNoDelay()
  auto sub24 = node->create_subscription<sensor_msgs::msg::NavSatFix>("fix", rclcpp::QoS(10), eagleye_fix_callback); //ros::TransportHints().tcpNoDelay()
  auto sub25 = node->create_subscription<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS(10), eagleye_twist_callback); //ros::TransportHints().tcpNoDelay()

  double delta_time = 1.0 / static_cast<double>(update_rate);
  auto timer_callback = std::bind(on_timer);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  auto timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    node->get_clock(), period_ns, std::move(timer_callback),
    node->get_node_base_interface()->get_context());
  node->get_node_timers_interface()->add_timer(timer, nullptr);
  rclcpp::spin(node);

  return 0;
}