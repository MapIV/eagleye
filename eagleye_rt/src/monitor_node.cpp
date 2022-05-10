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

static sensor_msgs::Imu _imu;
static rtklib_msgs::RtklibNav _rtklib_nav;
static sensor_msgs::NavSatFix _rtklib_fix;
static nmea_msgs::Gpgga _gga;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
static eagleye_msgs::Distance _distance;
static eagleye_msgs::Heading _heading_1st;
static eagleye_msgs::Heading _heading_interpolate_1st;
static eagleye_msgs::Heading _heading_2nd;
static eagleye_msgs::Heading _heading_interpolate_2nd;
static eagleye_msgs::Heading _heading_3rd;
static eagleye_msgs::Heading _heading_interpolate_3rd;
static eagleye_msgs::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::YawrateOffset _yawrate_offset_1st;
static eagleye_msgs::YawrateOffset _yawrate_offset_2nd;
static eagleye_msgs::SlipAngle _slip_angle;
static eagleye_msgs::Height _height;
static eagleye_msgs::Pitching _pitching;
static eagleye_msgs::Position _enu_relative_pos;
static geometry_msgs::Vector3Stamped _enu_vel;
static eagleye_msgs::Position _enu_absolute_pos;
static eagleye_msgs::Position _enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix _eagleye_fix;
static geometry_msgs::TwistStamped _eagleye_twist;

static geometry_msgs::TwistStamped::ConstPtr _comparison_velocity_ptr;
static sensor_msgs::Imu _corrected_imu;

static bool _gga_sub_status;
static bool _print_status;

static double _imu_time_last;
static double _rtklib_nav_time_last;
static double _navsat_gga_time_last;
static double _velocity_time_last;
static double _velocity_scale_factor_time_last;
static double _distance_time_last;
static double _heading_1st_time_last;
static double _heading_interpolate_1st_time_last;
static double _heading_2nd_time_last;
static double _heading_interpolate_2nd_time_last;
static double _heading_3rd_time_last;
static double _heading_interpolate_3rd_time_last;
static double _yawrate_offset_stop_time_last;
static double _yawrate_offset_1st_time_last;
static double _yawrate_offset_2nd_time_last;
static double _slip_angle_time_last;
static double _height_time_last;
static double _pitching_time_last;
static double _enu_vel_time_last;
static double _enu_absolute_pos_time_last;
static double _enu_absolute_pos_interpolate_time_last;
static double _eagleye_twist_time_last;

bool _use_compare_yawrate = false;
double _update_rate = 10.0;
double _th_gnss_deadrock_time = 10;
double _th_velocity_scale_factor_percent = 20;
double _th_diff_rad_per_sec = 0.17453;
int _num_continuous_abnormal_yawrate = 0;
int _th_num_continuous_abnormal_yawrate = 10;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  _rtklib_nav = *msg;
}

void rtklib_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  _rtklib_fix = *msg;
}

void navsatfix_gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  _gga = *msg;
  _gga_sub_status = true;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
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

void heading_1st_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_1st = *msg;
}

void heading_interpolate_1st_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_1st = *msg;
}

void heading_2nd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_2nd = *msg;
}

void heading_interpolate_2nd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_2nd = *msg;
}

void heading_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_3rd = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_3rd = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_stop = *msg;
}

void yawrate_offset_1st_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_1st = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yawrate_offset_2nd = *msg;
}

void slip_angle_callback(const eagleye_msgs::SlipAngle::ConstPtr& msg)
{
  _slip_angle = *msg;
}

void enu_relative_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _enu_relative_pos = *msg;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  _enu_vel = *msg;
}

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _enu_absolute_pos = *msg;
}

void height_callback(const eagleye_msgs::Height::ConstPtr& msg)
{
  _height = *msg;
}

void pitching_callback(const eagleye_msgs::Pitching::ConstPtr& msg)
{
  _pitching = *msg;
}

void enu_absolute_pos_interpolate_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _enu_absolute_pos_interpolate = *msg;
}

void eagleye_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  _eagleye_fix = *msg;
}

void eagleye_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _eagleye_twist = *msg;
}

void comparison_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _comparison_velocity_ptr = msg;
}

void corrected_imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _corrected_imu = *msg;
}

void imu_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_imu_time_last == _imu.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }

  _imu_time_last = _imu.header.stamp.toSec();
  stat.summary(level, msg);
}

void rtklib_nav_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_rtklib_nav_time_last - _rtklib_nav.header.stamp.toSec() > _th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }

  _rtklib_nav_time_last = _rtklib_nav.header.stamp.toSec();
  stat.summary(level, msg);
}

void navsat_fix_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_navsat_gga_time_last - _gga.header.stamp.toSec() > _th_gnss_deadrock_time || !_gga_sub_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  _navsat_gga_time_last = _gga.header.stamp.toSec();
  stat.summary(level, msg);
}

void velocity_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_velocity_time_last == _velocity.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::STALE;
    msg = "not subscribed to topic";
  }

  _velocity_time_last = _velocity.header.stamp.toSec();
  stat.summary(level, msg);
}

void velocity_scale_factor_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_velocity_scale_factor_time_last == _velocity_scale_factor.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_velocity_scale_factor.scale_factor)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_th_velocity_scale_factor_percent / 100 < std::abs(1.0 - _velocity_scale_factor.scale_factor)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Estimated velocity scale factor is too large or too small";
  }
  else if (!_velocity_scale_factor.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _velocity_scale_factor_time_last = _velocity_scale_factor.header.stamp.toSec();
  stat.summary(level, msg);
}

void distance_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_distance_time_last == _distance.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_distance.distance)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_distance.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _distance_time_last = _distance.header.stamp.toSec();
  stat.summary(level, msg);
}

void heading_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(_heading_1st.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_heading_1st_time_last - _heading_1st.header.stamp.toSec() > _th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!_heading_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _heading_1st_time_last = _heading_1st.header.stamp.toSec();
  stat.summary(level, msg);
}

void heading_interpolate_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_heading_interpolate_1st_time_last == _heading_interpolate_1st.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_heading_interpolate_1st.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_heading_interpolate_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _heading_interpolate_1st_time_last = _heading_interpolate_1st.header.stamp.toSec();
  stat.summary(level, msg);
}

void heading_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(_heading_2nd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_heading_2nd_time_last - _heading_2nd.header.stamp.toSec() > _th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!_heading_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _heading_2nd_time_last = _heading_2nd.header.stamp.toSec();
  stat.summary(level, msg);
}

void heading_interpolate_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_heading_interpolate_2nd_time_last == _heading_interpolate_2nd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_heading_interpolate_2nd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_heading_interpolate_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _heading_interpolate_2nd_time_last = _heading_interpolate_2nd.header.stamp.toSec();
  stat.summary(level, msg);
}

void heading_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(_heading_3rd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_heading_3rd_time_last - _heading_3rd.header.stamp.toSec() > _th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!_heading_3rd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _heading_3rd_time_last = _heading_3rd.header.stamp.toSec();
  stat.summary(level, msg);
}

void heading_interpolate_3rd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_heading_interpolate_3rd_time_last == _heading_interpolate_3rd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_heading_interpolate_3rd.heading_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_heading_interpolate_3rd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _heading_interpolate_3rd_time_last = _heading_interpolate_3rd.header.stamp.toSec();
  stat.summary(level, msg);
}
void yawrate_offset_stop_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_yawrate_offset_stop_time_last == _yawrate_offset_stop.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_yawrate_offset_stop.yawrate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_yawrate_offset_stop.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _yawrate_offset_stop_time_last = _yawrate_offset_stop.header.stamp.toSec();
  stat.summary(level, msg);
}

void yawrate_offset_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_yawrate_offset_1st_time_last == _yawrate_offset_1st.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_yawrate_offset_1st.yawrate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_yawrate_offset_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _yawrate_offset_1st_time_last = _yawrate_offset_1st.header.stamp.toSec();
  stat.summary(level, msg);
}

void yawrate_offset_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_yawrate_offset_2nd_time_last == _yawrate_offset_2nd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_yawrate_offset_2nd.yawrate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_yawrate_offset_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _yawrate_offset_2nd_time_last = _yawrate_offset_2nd.header.stamp.toSec();
  stat.summary(level, msg);
}

void slip_angle_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_slip_angle_time_last == _slip_angle.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_slip_angle.slip_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_slip_angle.coefficient == 0) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "/slip_angle/manual_coefficient is not set";
  }
  else if (!_slip_angle.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _slip_angle_time_last = _slip_angle.header.stamp.toSec();
  stat.summary(level, msg);
}

void enu_vel_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

 if (!std::isfinite(_enu_vel.vector.x)||!std::isfinite(_enu_vel.vector.y)||!std::isfinite(_enu_vel.vector.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else  if (_enu_vel_time_last == _enu_vel.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  _enu_vel_time_last = _enu_vel.header.stamp.toSec();
  stat.summary(level, msg);
}

void height_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_height_time_last == _height.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_height.height)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_height.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _height_time_last = _height.header.stamp.toSec();
  stat.summary(level, msg);
}

void pitching_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_pitching_time_last == _pitching.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_pitching.pitching_angle)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_pitching.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _pitching_time_last = _pitching.header.stamp.toSec();
  stat.summary(level, msg);
}

void enu_absolute_pos_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(_enu_absolute_pos.enu_pos.x)||!std::isfinite(_enu_absolute_pos.enu_pos.y)||!std::isfinite(_enu_absolute_pos.enu_pos.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_enu_absolute_pos_time_last - _enu_absolute_pos.header.stamp.toSec() > _th_gnss_deadrock_time) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!_enu_absolute_pos.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _enu_absolute_pos_time_last = _enu_absolute_pos.header.stamp.toSec();
  stat.summary(level, msg);
}

void enu_absolute_pos_interpolate_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!std::isfinite(_enu_absolute_pos_interpolate.enu_pos.x) || !std::isfinite(_enu_absolute_pos_interpolate.enu_pos.y) ||
    !std::isfinite(_enu_absolute_pos_interpolate.enu_pos.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (_enu_absolute_pos_interpolate_time_last == _enu_absolute_pos_interpolate.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!_enu_absolute_pos_interpolate.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  _enu_absolute_pos_interpolate_time_last = _enu_absolute_pos_interpolate.header.stamp.toSec();
  stat.summary(level, msg);
}

void twist_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_eagleye_twist_time_last == _eagleye_twist.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "not subscribed or deadlock of more than 10 seconds";
  }
  else if (!std::isfinite(_eagleye_twist.twist.linear.x)||!std::isfinite(_eagleye_twist.twist.linear.y)||!std::isfinite(_eagleye_twist.twist.linear.z)
      ||!std::isfinite(_eagleye_twist.twist.angular.x)||!std::isfinite(_eagleye_twist.twist.angular.y)||!std::isfinite(_eagleye_twist.twist.angular.z)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }

  _eagleye_twist_time_last = _eagleye_twist.header.stamp.toSec();
  stat.summary(level, msg);
}

void corrected_imu_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if(_comparison_velocity_ptr == nullptr)
  {
    return;
  }

  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if(_use_compare_yawrate && _th_diff_rad_per_sec < std::abs(_corrected_imu.angular_velocity.z - _comparison_velocity_ptr->twist.angular.z))
  {
    _num_continuous_abnormal_yawrate++;
  }
  else
  {
    _num_continuous_abnormal_yawrate = 0;
  }

  if (_num_continuous_abnormal_yawrate > _th_num_continuous_abnormal_yawrate) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Corrected yaw rate too large or too small";
  }
  stat.summary(level, msg);
}

void printStatus(void)
{
  std::cout << std::endl;
  std::cout<<"\033[1;33m Eagleye status \033[m"<<std::endl;
  std::cout << std::endl;
  std::cout << std::fixed;

  std::cout << "--- \033[1;34m imu(input)\033[m ------------------------------"<< std::endl;
  std::cout<<"\033[1m linear_acceleration \033[mx "<<std::setprecision(6)<<_imu.linear_acceleration.x<<" [m/s^2]"<<std::endl;
  std::cout<<"\033[1m linear acceleration \033[my "<<std::setprecision(6)<<_imu.linear_acceleration.y<<" [m/s^2]"<<std::endl;
  std::cout<<"\033[1m linear acceleration \033[mz "<<std::setprecision(6)<<_imu.linear_acceleration.z<<" [m/s^2]"<<std::endl;
  std::cout<<"\033[1m angular velocity \033[mx "<<std::setprecision(6)<<_imu.angular_velocity.x<<" [rad/s]"<<std::endl;
  std::cout<<"\033[1m angular velocity \033[my "<<std::setprecision(6)<<_imu.angular_velocity.y<<" [rad/s]"<<std::endl;
  std::cout<<"\033[1m angular velocity \033[mz "<<std::setprecision(6)<<_imu.angular_velocity.z<<" [rad/s]"<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m velocity(input)\033[m -------------------------"<< std::endl;
  std::cout<<"\033[1m velocity \033[m"<<std::setprecision(4)<<_velocity.twist.linear.x * 3.6<<" [km/h]"<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m rtklib(input)\033[m ---------------------------"<< std::endl;
  std::cout<<"\033[1m time of week  \033[m"<<_rtklib_nav.tow<<" [ms]"<<std::endl;
  std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_rtklib_nav.status.latitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_rtklib_nav.status.longitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_rtklib_nav.status.altitude<<" [m]"<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m gga(input)\033[m ------------------------------"<< std::endl;

  if(_gga_sub_status)
  {
    std::cout<< "\033[1m rtk status \033[m "<<int(_gga.gps_qual)<<std::endl;
    std::cout<< "\033[1m rtk status \033[m "<<(int(_gga.gps_qual)!=4 ? "\033[1;31mNo Fix\033[m" : "\033[1;32mFix\033[m")<<std::endl;
    std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_gga.lat<<" [deg]"<<std::endl;
    std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_gga.lon<<" [deg]"<<std::endl;
    std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_gga.alt + _gga.undulation<<" [m]"<<std::endl;
    std::cout << std::endl;
  }
  else
  {
    std::cout << std::endl;
    std::cout<<"\033[1;31m no subscription \033[m"<<std::endl;
    std::cout << std::endl;
  }


  std::cout << "--- \033[1;34m velocity SF\033[m -----------------------------"<< std::endl;
  std::cout<<"\033[1m scale factor \033[m "<<std::setprecision(4)<<_velocity_scale_factor.scale_factor<<std::endl;
  std::cout<<"\033[1m correction velocity \033[m "<<std::setprecision(4)<<_velocity_scale_factor.correction_velocity.linear.x * 3.6<<" [km/h]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_velocity_scale_factor.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m yawrate offset stop\033[m ---------------------"<< std::endl;
  std::cout<<"\033[1m yawrate offset \033[m "<<std::setprecision(6)<<_yawrate_offset_stop.yawrate_offset<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_yawrate_offset_stop.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m yawrate offset\033[m --------------------------"<< std::endl;
  std::cout<<"\033[1m yawrate offset \033[m "<<std::setprecision(6)<<_yawrate_offset_2nd.yawrate_offset<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_yawrate_offset_2nd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m slip angle\033[m ------------------------------"<< std::endl;
  std::cout<<"\033[1m coefficient \033[m "<<std::setprecision(6)<<_slip_angle.coefficient<<std::endl;
  std::cout<<"\033[1m slip angle \033[m "<<std::setprecision(6)<<_slip_angle.slip_angle<<" [rad]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_slip_angle.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m heading\033[m ---------------------------------"<< std::endl;
  std::cout<<"\033[1m heading \033[m "<<std::setprecision(6)<<_heading_interpolate_3rd.heading_angle<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_heading_interpolate_3rd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m pitching\033[m --------------------------------"<< std::endl;
  std::cout<<"\033[1m pitching \033[m "<<std::setprecision(6)<<_pitching.pitching_angle<<" [rad]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_pitching.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m height\033[m ----------------------------------"<< std::endl;
  std::cout<<"\033[1m height \033[m "<<std::setprecision(4)<<_height.height<<" [m]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_height.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m position\033[m --------------------------------"<< std::endl;
  std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_eagleye_fix.latitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_eagleye_fix.longitude<<" [deg]"<<std::endl;
  std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_eagleye_fix.altitude<<" [m]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_enu_absolute_pos_interpolate.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;
}

void timer_callback(const ros::TimerEvent& e, diagnostic_updater::Updater * updater)
{
  // Diagnostic Updater
  updater->force_update();

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu.header = msg->header;
  _imu.orientation = msg->orientation;
  _imu.orientation_covariance = msg->orientation_covariance;
  _imu.angular_velocity = msg->angular_velocity;
  _imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  _imu.linear_acceleration = msg->linear_acceleration;
  _imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;

  if(_print_status)
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
  std::string subscribe_gga_topic_name = "/navsat/gga";
  std::string comparison_twist_topic_name = "/calculated_twist";

  n.getParam("twist_topic",subscribe_twist_topic_name);
  n.getParam("imu_topic",subscribe_imu_topic_name);
  n.getParam("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("monitor/print_status",_print_status);
  n.getParam("monitor/update_rate",_update_rate);
  n.getParam("monitor/th_gnss_deadrock_time",_th_gnss_deadrock_time);
  n.getParam("monitor/th_velocity_scale_factor_percent",_th_velocity_scale_factor_percent);
  n.getParam("monitor/use_compare_yawrate",_use_compare_yawrate);
  n.getParam("monitor/comparison_twist_topic",comparison_twist_topic_name);
  n.getParam("monitor/th_diff_rad_per_sec",_th_diff_rad_per_sec);
  n.getParam("monitor/th_num_continuous_abnormal_yawrate",_th_num_continuous_abnormal_yawrate);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_gga_topic_name "<<subscribe_gga_topic_name<<std::endl;
  std::cout<< "print_status "<<_print_status<<std::endl;
  std::cout<< "update_rate "<<_update_rate<<std::endl;
  std::cout<< "th_gnss_deadrock_time "<<_th_gnss_deadrock_time<<std::endl;
  std::cout<< "th_velocity_scale_factor_percent "<<_th_velocity_scale_factor_percent<<std::endl;
  std::cout<< "use_compare_yawrate "<<_use_compare_yawrate<<std::endl;
  std::cout<< "comparison_twist_topic_name "<<comparison_twist_topic_name<<std::endl;
  std::cout<< "th_diff_rad_per_sec "<<_th_diff_rad_per_sec<<std::endl;

  // // Diagnostic Updater
  diagnostic_updater::Updater updater;
  updater.setHardwareID("topic_checker");
  updater.add("eagleye_input_imu", imu_topic_checker);
  updater.add("eagleye_input_rtklib_nav", rtklib_nav_topic_checker);
  updater.add("eagleye_input_navsat_fix", navsat_fix_topic_checker);
  updater.add("eagleye_input_velocity", velocity_topic_checker);
  updater.add("eagleye_velocity_scale_factor", velocity_scale_factor_topic_checker);
  updater.add("eagleye_distance", distance_topic_checker);
  updater.add("eagleye_heading_1st", heading_1st_topic_checker);
  updater.add("eagleye_heading_interpolate_1st", heading_interpolate_1st_topic_checker);
  updater.add("eagleye_heading_2nd", heading_2nd_topic_checker);
  updater.add("eagleye_heading_interpolate_2nd", heading_interpolate_2nd_topic_checker);
  updater.add("eagleye_heading_3rd", heading_3rd_topic_checker);
  updater.add("eagleye_heading_interpolate_3rd", heading_interpolate_3rd_topic_checker);
  updater.add("eagleye_yawrate_offset_stop", yawrate_offset_stop_topic_checker);
  updater.add("eagleye_yawrate_offset_1st", yawrate_offset_1st_topic_checker);
  updater.add("eagleye_yawrate_offset_2nd", yawrate_offset_2nd_topic_checker);
  updater.add("eagleye_slip_angle", slip_angle_topic_checker);
  updater.add("eagleye_enu_vel", enu_vel_topic_checker);
  updater.add("eagleye_height", height_topic_checker);
  updater.add("eagleye_pitching", pitching_topic_checker);
  updater.add("eagleye_enu_absolute_pos", enu_absolute_pos_topic_checker);
  updater.add("eagleye_enu_absolute_pos_interpolate", enu_absolute_pos_interpolate_topic_checker);
  updater.add("eagleye_twist", twist_topic_checker);
  updater.add("eagleye_corrected_imu", corrected_imu_topic_checker);

  ros::Subscriber sub1 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("rtklib/fix", 1000, rtklib_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_gga_topic_name, 1000, navsatfix_gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = n.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub8 = n.subscribe("heading_1st", 1000, heading_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub9 = n.subscribe("heading_interpolate_1st", 1000, heading_interpolate_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub10 = n.subscribe("heading_2nd", 1000, heading_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub11 = n.subscribe("heading_interpolate_2nd", 1000, heading_interpolate_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub12 = n.subscribe("heading_3rd", 1000, heading_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub13 = n.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub14 = n.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub15 = n.subscribe("yawrate_offset_1st", 1000, yawrate_offset_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub16 = n.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub17 = n.subscribe("slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub18 = n.subscribe("enu_relative_pos", 1000, enu_relative_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub19 = n.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub20 = n.subscribe("height", 1000, height_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub21 = n.subscribe("pitching", 1000, pitching_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub22 = n.subscribe("enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub23 = n.subscribe("enu_absolute_pos_interpolate", 1000, enu_absolute_pos_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub24 = n.subscribe("fix", 1000, eagleye_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub25 = n.subscribe("twist", 1000, eagleye_twist_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub26 = n.subscribe(comparison_twist_topic_name, 1000, comparison_velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub27 = n.subscribe("imu/data_corrected", 1000, corrected_imu_callback, ros::TransportHints().tcpNoDelay());

  ros::Timer timer = n.createTimer(ros::Duration(1/_update_rate), boost::bind(timer_callback,_1, &updater));

  ros::spin();

  return 0;
}