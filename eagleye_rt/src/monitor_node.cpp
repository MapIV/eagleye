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

#include <boost/bind.hpp>
#include <diagnostic_updater/diagnostic_updater.h>

static sensor_msgs::Imu imu;
static rtklib_msgs::RtklibNav rtklib_nav;
static sensor_msgs::NavSatFix fix;
static sensor_msgs::NavSatFix navsat_fix;
static geometry_msgs::TwistStamped velocity;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::Distance distance;
static eagleye_msgs::Heading heading_1st;
static eagleye_msgs::Heading heading_interpolate_1st;
static eagleye_msgs::Heading heading_2nd;
static eagleye_msgs::Heading heading_interpolate_2nd;
static eagleye_msgs::Heading heading_3rd;
static eagleye_msgs::Heading heading_interpolate_3rd;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::YawrateOffset yawrate_offset_1st;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::SlipAngle slip_angle;
static eagleye_msgs::Height height;
static eagleye_msgs::Pitching pitching;
static eagleye_msgs::Position enu_relative_pos;
static geometry_msgs::Vector3Stamped enu_vel;
static eagleye_msgs::Position enu_absolute_pos;
static eagleye_msgs::Position enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix eagleye_fix;
static geometry_msgs::TwistStamped eagleye_twist;

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
static double enu_relative_pos_time_last;
static double enu_vel_time_last;
static double enu_absolute_pos_time_last;
static double enu_absolute_pos_interpolate_time_last;
static double eagleye_fix_time_last;
static double eagleye_twist_time_last;

static double update_rate = 10;
static double th_gnss_deadrock_time = 10;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

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

void navsatfix_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  navsat_fix.header = msg->header;
  navsat_fix.status = msg->status;
  navsat_fix.latitude = msg->latitude;
  navsat_fix.longitude = msg->longitude;
  navsat_fix.altitude = msg->altitude;
  navsat_fix.position_covariance = msg->position_covariance;
  navsat_fix.position_covariance_type = msg->position_covariance_type;
  navsat_fix_sub_status = true;
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

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}

void heading_1st_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_1st.header = msg->header;
  heading_1st.heading_angle = msg->heading_angle;
  heading_1st.status = msg->status;
}

void heading_interpolate_1st_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_1st.header = msg->header;
  heading_interpolate_1st.heading_angle = msg->heading_angle;
  heading_interpolate_1st.status = msg->status;
}

void heading_2nd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_2nd.header = msg->header;
  heading_2nd.heading_angle = msg->heading_angle;
  heading_2nd.status = msg->status;
}

void heading_interpolate_2nd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_2nd.header = msg->header;
  heading_interpolate_2nd.heading_angle = msg->heading_angle;
  heading_interpolate_2nd.status = msg->status;
}

void heading_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_3rd.header = msg->header;
  heading_3rd.heading_angle = msg->heading_angle;
  heading_3rd.status = msg->status;
}

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

void yawrate_offset_1st_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_1st.header = msg->header;
  yawrate_offset_1st.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_1st.status = msg->status;
}

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

void enu_relative_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_relative_pos.header = msg->header;
  enu_relative_pos.enu_pos = msg->enu_pos;
  enu_relative_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_relative_pos.status = msg->status;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_vel.header = msg->header;
  enu_vel.vector = msg->vector;
}

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}

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

void eagleye_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  eagleye_twist.header = msg->header;
  eagleye_twist.twist = msg->twist;
}

void imu_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (imu_time_last == imu.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }

  imu_time_last = imu.header.stamp.toSec();
  stat.summary(level, msg);
}
void rtklib_nav_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (rtklib_nav_time_last - rtklib_nav.header.stamp.toSec() > th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }

  rtklib_nav_time_last = rtklib_nav.header.stamp.toSec();
  stat.summary(level, msg);
}
void navsat_fix_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (navsat_fix_time_last - navsat_fix.header.stamp.toSec() > th_gnss_deadrock_time || !navsat_fix_sub_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  navsat_fix_time_last = navsat_fix.header.stamp.toSec();
  stat.summary(level, msg);
}
void velocity_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (velocity_time_last == velocity.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  velocity_time_last = velocity.header.stamp.toSec();
  stat.summary(level, msg);
}
void velocity_scale_factor_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (velocity_scale_factor_time_last == velocity_scale_factor.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(velocity_scale_factor.scale_factor)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!velocity_scale_factor.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  velocity_scale_factor_time_last = velocity_scale_factor.header.stamp.toSec();
  stat.summary(level, msg);
}
void distance_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (distance_time_last == distance.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(distance.distance)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!distance.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  distance_time_last = distance.header.stamp.toSec();
  stat.summary(level, msg);
}
void heading_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(heading_1st.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (heading_1st_time_last - heading_1st.header.stamp.toSec() > th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!heading_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_1st_time_last = heading_1st.header.stamp.toSec();
  stat.summary(level, msg);
}
void heading_interpolate_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (heading_interpolate_1st_time_last == heading_interpolate_1st.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(heading_interpolate_1st.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!heading_interpolate_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_interpolate_1st_time_last = heading_interpolate_1st.header.stamp.toSec();
  stat.summary(level, msg);
}
void heading_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(heading_2nd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (heading_2nd_time_last - heading_2nd.header.stamp.toSec() > th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!heading_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_2nd_time_last = heading_2nd.header.stamp.toSec();
  stat.summary(level, msg);
}
void heading_interpolate_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (heading_interpolate_2nd_time_last == heading_interpolate_2nd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(heading_interpolate_2nd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!heading_interpolate_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_interpolate_2nd_time_last = heading_interpolate_2nd.header.stamp.toSec();
  stat.summary(level, msg);
}
void heading_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(heading_3rd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (heading_3rd_time_last - heading_3rd.header.stamp.toSec() > th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!heading_3rd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_3rd_time_last = heading_3rd.header.stamp.toSec();
  stat.summary(level, msg);
}
void heading_interpolate_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (heading_interpolate_3rd_time_last == heading_interpolate_3rd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(heading_interpolate_3rd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!heading_interpolate_3rd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  heading_interpolate_3rd_time_last = heading_interpolate_3rd.header.stamp.toSec();
  stat.summary(level, msg);
}
void yawrate_offset_stop_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (yawrate_offset_stop_time_last == yawrate_offset_stop.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(yawrate_offset_stop.yawrate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!yawrate_offset_stop.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  yawrate_offset_stop_time_last = yawrate_offset_stop.header.stamp.toSec();
  stat.summary(level, msg);
}
void yawrate_offset_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (yawrate_offset_1st_time_last == yawrate_offset_1st.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(yawrate_offset_1st.yawrate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!yawrate_offset_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  yawrate_offset_1st_time_last = yawrate_offset_1st.header.stamp.toSec();
  stat.summary(level, msg);
}
void yawrate_offset_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (yawrate_offset_2nd_time_last == yawrate_offset_2nd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(yawrate_offset_2nd.yawrate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!yawrate_offset_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  yawrate_offset_2nd_time_last = yawrate_offset_2nd.header.stamp.toSec();
  stat.summary(level, msg);
}
void slip_angle_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (slip_angle_time_last == slip_angle.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(slip_angle.slip_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (slip_angle.coefficient == 0) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "/slip_angle/manual_coefficient is not set";
  }
  else if (!slip_angle.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  slip_angle_time_last = slip_angle.header.stamp.toSec();
  stat.summary(level, msg);
}
void enu_vel_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

 if (!std::isfinite(enu_vel.vector.x)&&!std::isfinite(enu_vel.vector.y)&&!std::isfinite(enu_vel.vector.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else  if (enu_vel_time_last == enu_vel.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  enu_vel_time_last = enu_vel.header.stamp.toSec();
  stat.summary(level, msg);
}
void height_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (height_time_last == height.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(height.height)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!height.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  height_time_last = height.header.stamp.toSec();
  stat.summary(level, msg);
}
void pitching_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (pitching_time_last == pitching.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(pitching.pitching_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!pitching.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  pitching_time_last = pitching.header.stamp.toSec();
  stat.summary(level, msg);
}
void enu_absolute_pos_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(enu_absolute_pos.enu_pos.x)&&!std::isfinite(enu_absolute_pos.enu_pos.y)&&!std::isfinite(enu_absolute_pos.enu_pos.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (enu_absolute_pos_time_last - enu_absolute_pos.header.stamp.toSec() > th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!enu_absolute_pos.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  enu_absolute_pos_time_last = enu_absolute_pos.header.stamp.toSec();
  stat.summary(level, msg);
}
void enu_absolute_pos_interpolate_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(enu_absolute_pos_interpolate.enu_pos.x)&&!std::isfinite(enu_absolute_pos_interpolate.enu_pos.y)&&!std::isfinite(enu_absolute_pos_interpolate.enu_pos.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (enu_absolute_pos_interpolate_time_last == enu_absolute_pos_interpolate.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!enu_absolute_pos_interpolate.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  enu_absolute_pos_interpolate_time_last = enu_absolute_pos_interpolate.header.stamp.toSec();
  stat.summary(level, msg);
}
void twist_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (eagleye_twist_time_last == eagleye_twist.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(eagleye_twist.twist.linear.x)&&!std::isfinite(eagleye_twist.twist.linear.y)&&!std::isfinite(eagleye_twist.twist.linear.z)
      &&!std::isfinite(eagleye_twist.twist.angular.x)&&!std::isfinite(eagleye_twist.twist.angular.y)&&!std::isfinite(eagleye_twist.twist.angular.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }

  eagleye_twist_time_last = eagleye_twist.header.stamp.toSec();
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

  if(navsat_fix_sub_status)
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

void timer_callback(const ros::TimerEvent& e, diagnostic_updater::Updater * updater_)
{
  // Diagnostic Updater
  updater_->force_update();

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  if(print_status)
  {
    printStatus();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "monitor");

  ros::NodeHandle n;

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_navsatfix_topic_name = "/navsatfix/fix";

  n.getParam("eagleye/twist_topic",subscribe_twist_topic_name);
  n.getParam("eagleye/imu_topic",subscribe_imu_topic_name);
  n.getParam("eagleye/rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("eagleye/navsatfix_topic",subscribe_navsatfix_topic_name);
  n.getParam("eagleye/monitor/print_status",print_status);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;
  std::cout<< "print_status "<<print_status<<std::endl;


  // // Diagnostic Updater
  diagnostic_updater::Updater updater_;
  updater_.setHardwareID("topic_checker");
  updater_.add("imu", imu_topic_checker);
  updater_.add("rtklib_nav", rtklib_nav_topic_checker);
  updater_.add("navsat_fix", navsat_fix_topic_checker);
  updater_.add("velocity", velocity_topic_checker);
  updater_.add("velocity_scale_factor", velocity_scale_factor_topic_checker);
  updater_.add("distance", distance_topic_checker);
  updater_.add("heading_1st", heading_1st_topic_checker);
  updater_.add("heading_interpolate_1st", heading_interpolate_1st_topic_checker);
  updater_.add("heading_2nd", heading_2nd_topic_checker);
  updater_.add("heading_interpolate_2nd", heading_interpolate_2nd_topic_checker);
  updater_.add("heading_3rd", heading_3rd_topic_checker);
  updater_.add("heading_interpolate_3rd", heading_interpolate_3rd_topic_checker);
  updater_.add("yawrate_offset_stop", yawrate_offset_stop_topic_checker);
  updater_.add("yawrate_offset_1st", yawrate_offset_1st_topic_checker);
  updater_.add("yawrate_offset_2nd", yawrate_offset_2nd_topic_checker);
  updater_.add("slip_angle", slip_angle_topic_checker);
  updater_.add("enu_vel", enu_vel_topic_checker);
  updater_.add("height", height_topic_checker);
  updater_.add("pitching", pitching_topic_checker);
  updater_.add("enu_absolute_pos", enu_absolute_pos_topic_checker);
  updater_.add("enu_absolute_pos_interpolate", enu_absolute_pos_interpolate_topic_checker);
  updater_.add("twist", twist_topic_checker);

  ros::Subscriber sub1 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("f9p/fix", 1000, fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_navsatfix_topic_name, 1000, navsatfix_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = n.subscribe("eagleye/distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub8 = n.subscribe("eagleye/heading_1st", 1000, heading_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub9 = n.subscribe("eagleye/heading_interpolate_1st", 1000, heading_interpolate_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub10 = n.subscribe("eagleye/heading_2nd", 1000, heading_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub11 = n.subscribe("eagleye/heading_interpolate_2nd", 1000, heading_interpolate_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub12 = n.subscribe("eagleye/heading_3rd", 1000, heading_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub13 = n.subscribe("eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub14 = n.subscribe("eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub15 = n.subscribe("eagleye/yawrate_offset_1st", 1000, yawrate_offset_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub16 = n.subscribe("eagleye/yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub17 = n.subscribe("eagleye/slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub18 = n.subscribe("eagleye/enu_relative_pos", 1000, enu_relative_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub19 = n.subscribe("eagleye/enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub20 = n.subscribe("eagleye/height", 1000, height_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub21 = n.subscribe("eagleye/pitching", 1000, pitching_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub22 = n.subscribe("eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub23 = n.subscribe("eagleye/enu_absolute_pos_interpolate", 1000, enu_absolute_pos_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub24 = n.subscribe("eagleye/fix", 1000, eagleye_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub25 = n.subscribe("eagleye/twist", 1000, eagleye_twist_callback, ros::TransportHints().tcpNoDelay());

  ros::Timer timer = n.createTimer(ros::Duration(1/update_rate), boost::bind(timer_callback,_1, &updater_));

  ros::spin();

  return 0;
}
