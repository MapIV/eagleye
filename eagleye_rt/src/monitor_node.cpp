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
#include <ros/package.h>

static sensor_msgs::Imu _imu;
static rtklib_msgs::RtklibNav _rtklib_nav;
static sensor_msgs::NavSatFix _rtklib_fix;
static nmea_msgs::Gpgga::ConstPtr _gga_ptr;
static geometry_msgs::TwistStamped _velocity;
static geometry_msgs::TwistStamped _correction_velocity;
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
eagleye_msgs::Distance::ConstPtr _distance_ptr;
static eagleye_msgs::Heading _heading_1st;
static eagleye_msgs::Heading _heading_interpolate_1st;
static eagleye_msgs::Heading _heading_2nd;
static eagleye_msgs::Heading _heading_interpolate_2nd;
static eagleye_msgs::Heading _heading_3rd;
static eagleye_msgs::Heading _heading_interpolate_3rd;
static eagleye_msgs::YawrateOffset _yaw_rate_offset_stop;
static eagleye_msgs::YawrateOffset _yaw_rate_offset_1st;
static eagleye_msgs::YawrateOffset _yaw_rate_offset_2nd;
static eagleye_msgs::SlipAngle _slip_angle;
static eagleye_msgs::Height _height;
static eagleye_msgs::Pitching _pitching;
static eagleye_msgs::Rolling _rolling;
static eagleye_msgs::Position _enu_relative_pos;
static geometry_msgs::Vector3Stamped _enu_vel;
static eagleye_msgs::Position _enu_absolute_pos;
static eagleye_msgs::Position _enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix _eagleye_fix;
static geometry_msgs::TwistStamped _eagleye_twist;

static geometry_msgs::TwistStamped::ConstPtr _comparison_velocity_ptr;
static sensor_msgs::Imu _corrected_imu;

static bool _gga_sub_status;
static bool _print_status, _log_output_status, _log_header_make = false;
static std::string output_log_dir;

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
static double _yaw_rate_offset_stop_time_last;
static double _yaw_rate_offset_1st_time_last;
static double _yaw_rate_offset_2nd_time_last;
static double _slip_angle_time_last;
static double _height_time_last;
static double _pitching_time_last;
static double _enu_vel_time_last;
static double _enu_absolute_pos_time_last;
static double _enu_absolute_pos_interpolate_time_last;
static double _eagleye_twist_time_last;

bool _use_compare_yaw_rate = false;
double _update_rate = 10.0;
double _th_gnss_deadrock_time = 10;
double _th_diff_rad_per_sec = 0.17453; // [rad/sec]
int _num_continuous_abnormal_yaw_rate = 0;
int _th_num_continuous_abnormal_yaw_rate = 10;

bool _use_rtk_dead_reckoning = false;
eagleye_msgs::Distance _previous_distance;
double _dr_distance = 0; // dead reckoning distance
double _th_dr_distance = 100.0; // [m]

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
  _gga_ptr = msg;
  _gga_sub_status = true;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _velocity = *msg;
}

void correction_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _correction_velocity = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  if (_distance_ptr != nullptr && _gga_ptr != nullptr){
    double delta_distance = _distance_ptr->distance - _previous_distance.distance;
    _dr_distance += delta_distance;
    if(int(_gga_ptr->gps_qual) == 4) {
      _dr_distance = 0;
    }
  }
  _distance_ptr = msg;
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

void yaw_rate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset_stop = *msg;
}

void yaw_rate_offset_1st_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset_1st = *msg;
}

void yaw_rate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset_2nd = *msg;
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

void rolling_callback(const eagleye_msgs::Rolling::ConstPtr& msg)
{
  _rolling = *msg;
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

  if (_gga_ptr == nullptr ||  _navsat_gga_time_last - _gga_ptr->header.stamp.toSec() > _th_gnss_deadrock_time || !_gga_sub_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }

  if (_gga_ptr != nullptr)  _navsat_gga_time_last = _gga_ptr->header.stamp.toSec();
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
  else if (!_velocity_scale_factor.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }
  else if (_velocity_scale_factor.status.is_abnormal) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    if (_velocity_scale_factor.status.error_code == eagleye_msgs::Status::NAN_OR_INFINITE)
    {
      msg = "Estimated velocity scale factor is NaN or infinete";
    }
    else if (_velocity_scale_factor.status.error_code == eagleye_msgs::Status::TOO_LARGE_OR_SMALL)
    {
      msg = "Estimated velocity scale factor is too large or too small";
    }
    else
    {
      msg = "abnormal error of velocity_scale_factor";
    }
  }

  _velocity_scale_factor_time_last = _velocity_scale_factor.header.stamp.toSec();
  stat.summary(level, msg);
}

void distance_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_distance_ptr == nullptr || _distance_time_last == _distance_ptr->header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_distance_ptr->distance)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_distance_ptr->status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }

  if (_distance_ptr != nullptr) _distance_time_last = _distance_ptr->header.stamp.toSec();
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
void yaw_rate_offset_stop_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_yaw_rate_offset_stop_time_last == _yaw_rate_offset_stop.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!_yaw_rate_offset_stop.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }
  else if (!_yaw_rate_offset_stop.status.is_abnormal) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    if(_yaw_rate_offset_stop.status.error_code == eagleye_msgs::Status::NAN_OR_INFINITE)
    {
      msg = "estimate value is NaN or infinete";
    }
    else
    {
      msg = "abnormal error of yaw_rate_offset_stop";
    }
  }

  _yaw_rate_offset_stop_time_last = _yaw_rate_offset_stop.header.stamp.toSec();
  stat.summary(level, msg);
}

void yaw_rate_offset_1st_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_yaw_rate_offset_1st_time_last == _yaw_rate_offset_1st.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!_yaw_rate_offset_1st.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }
  else if (!_yaw_rate_offset_1st.status.is_abnormal) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    if(_yaw_rate_offset_1st.status.error_code == eagleye_msgs::Status::NAN_OR_INFINITE)
    {
      msg = "estimate value is NaN or infinete";
    }
    else
    {
      msg = "abnormal error of yaw_rate_offset_1st";
    }
  }

  _yaw_rate_offset_1st_time_last = _yaw_rate_offset_1st.header.stamp.toSec();
  stat.summary(level, msg);
}

void yaw_rate_offset_2nd_topic_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (_yaw_rate_offset_2nd_time_last == _yaw_rate_offset_2nd.header.stamp.toSec()) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "not subscribed to topic";
  }
  else if (!std::isfinite(_yaw_rate_offset_2nd.yaw_rate_offset)) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "invalid number";
  }
  else if (!_yaw_rate_offset_2nd.status.enabled_status) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    msg = "estimates have not started yet";
  }
  else if (!_yaw_rate_offset_2nd.status.is_abnormal) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    if(_yaw_rate_offset_2nd.status.error_code == eagleye_msgs::Status::NAN_OR_INFINITE)
    {
      msg = "estimate value is NaN or infinete";
    }
    else
    {
      msg = "abnormal error of yaw_rate_offset_2nd";
    }
  }

  _yaw_rate_offset_2nd_time_last = _yaw_rate_offset_2nd.header.stamp.toSec();
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

  if(_use_compare_yaw_rate && _th_diff_rad_per_sec < std::abs(_corrected_imu.angular_velocity.z - _comparison_velocity_ptr->twist.angular.z))
  {
    _num_continuous_abnormal_yaw_rate++;
  }
  else
  {
    _num_continuous_abnormal_yaw_rate = 0;
  }

  if (_num_continuous_abnormal_yaw_rate > _th_num_continuous_abnormal_yaw_rate) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "Corrected yaw rate too large or too small";
  }
  stat.summary(level, msg);
}

void dr_distance_checker(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string msg = "OK";

  if(_th_dr_distance < _dr_distance)
  {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    msg = "The dead reckoning interval is too large";
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
    std::cout<< "\033[1m rtk status \033[m "<<int(_gga_ptr->gps_qual)<<std::endl;
    std::cout<< "\033[1m rtk status \033[m "<<(int(_gga_ptr->gps_qual)!=4 ? "\033[1;31mNo Fix\033[m" : "\033[1;32mFix\033[m")<<std::endl;
    std::cout<<"\033[1m latitude  \033[m"<<std::setprecision(8)<<_gga_ptr->lat<<" [deg]"<<std::endl;
    std::cout<<"\033[1m longitude  \033[m"<<std::setprecision(8)<<_gga_ptr->lon<<" [deg]"<<std::endl;
    std::cout<<"\033[1m altitude  \033[m"<<std::setprecision(4)<<_gga_ptr->alt + _gga_ptr->undulation<<" [m]"<<std::endl;
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
  std::cout<< "\033[1m status enable \033[m "<<(_velocity_scale_factor.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m yaw_rate offset stop\033[m ---------------------"<< std::endl;
  std::cout<<"\033[1m yaw_rate offset \033[m "<<std::setprecision(6)<<_yaw_rate_offset_stop.yaw_rate_offset<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_yaw_rate_offset_stop.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
  std::cout << std::endl;

  std::cout << "--- \033[1;34m yaw_rate offset\033[m --------------------------"<< std::endl;
  std::cout<<"\033[1m yaw_rate offset \033[m "<<std::setprecision(6)<<_yaw_rate_offset_2nd.yaw_rate_offset<<" [rad/s]"<<std::endl;
  std::cout<< "\033[1m status enable \033[m "<<(_yaw_rate_offset_2nd.status.enabled_status ? "\033[1;32mTrue\033[m" : "\033[1;31mFalse\033[m")<<std::endl;
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

void outputLog(void)
{
  if(!_log_header_make)
  {
  std::ofstream output_log_file(output_log_dir, std::ios_base::trunc | std::ios_base::out);
  std::cout << "Output file = eagleye_log.csv" << std::endl;
  output_log_file << "timestamp,imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z\
,rtklib_nav.tow,rtklib_nav.ecef_pos.x,rtklib_nav.ecef_pos.y,rtklib_nav.ecef_pos.z,rtklib_nav.ecef_vel.x,rtklib_nav.ecef_vel.y,rtklib_nav.ecef_vel.z,rtklib_nav.status.status.status,rtklib_nav.status.status.service,rtklib_nav.status.latitude,rtklib_nav.status.longitude,rtklib_nav.status.altitude\
,velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z,velocity.twist.angular.x,velocity.twist.angular.y,velocity.twist.angular.z\
,velocity_scale_factor.scale_factor,correction_velocity.twist.linear.x,correction_velocity.twist.linear.y,correction_velocity.twist.linear.z,correction_velocity.twist.angular.x,correction_velocity.twist.angular.y,correction_velocity.twist.angular.z,velocity_scale_factor.status.enabled_status,velocity_scale_factor.status.estimate_status\
,distance.distance,distance.status.enabled_status,distance.status.estimate_status\
,heading_1st.heading_angle,heading_1st.status.enabled_status,heading_1st.status.estimate_status\
,heading_interpolate_1st.heading_angle,heading_interpolate_1st.status.enabled_status,heading_interpolate_1st.status.estimate_status\
,heading_2nd.heading_angle,heading_2nd.status.enabled_status,heading_2nd.status.estimate_status\
,heading_interpolate_2nd.heading_angle,heading_interpolate_2nd.status.enabled_status,heading_interpolate_2nd.status.estimate_status\
,heading_3rd.heading_angle,heading_3rd.status.enabled_status,heading_3rd.status.estimate_status\
,heading_interpolate_3rd.heading_angle,heading_interpolate_3rd.status.enabled_status,heading_interpolate_3rd.status.estimate_status\
,yaw_rate_offset_stop.yaw_rate_offset,yaw_rate_offset_stop.status.enabled_status,yaw_rate_offset_stop.status.estimate_status\
,yaw_rate_offset_1st.yaw_rate_offset,yaw_rate_offset_1st.status.enabled_status,yaw_rate_offset_1st.status.estimate_status\
,yaw_rate_offset_2nd.yaw_rate_offset,yaw_rate_offset_2nd.status.enabled_status,yaw_rate_offset_2nd.status.estimate_status\
,slip_angle.coefficient,slip_angle.slip_angle,slip_angle.status.enabled_status,slip_angle.status.estimate_status\
,enu_vel.vector.x,enu_vel.vector.y,enu_vel.vector.z\
,enu_absolute_pos.enu_pos.x,enu_absolute_pos.enu_pos.y,enu_absolute_pos.enu_pos.z,enu_absolute_pos.ecef_base_pos.x,enu_absolute_pos.ecef_base_pos.y,enu_absolute_pos.ecef_base_pos.z,enu_absolute_pos.status.enabled_status,enu_absolute_pos.status.estimate_status\
,enu_absolute_pos_interpolate.enu_pos.x,enu_absolute_pos_interpolate.enu_pos.y,enu_absolute_pos_interpolate.enu_pos.z,enu_absolute_pos_interpolate.ecef_base_pos.x,enu_absolute_pos_interpolate.ecef_base_pos.y,enu_absolute_pos_interpolate.ecef_base_pos.z,enu_absolute_pos_interpolate.status.enabled_status,enu_absolute_pos_interpolate.status.estimate_status\
,height.height,height.status.enabled_status,height.status.estimate_status\
,pitching.pitching_angle,pitching.status.enabled_status,pitching.status.estimate_status\
,acc_x_offset.acc_x_offset,acc_x_offset.status.enabled_status,acc_x_offset.status.estimate_status\
,acc_x_scale_factor.acc_x_scale_factor,acc_x_scale_factor.status.enabled_status,acc_x_scale_factor.status.estimate_status\
,rolling.rolling_angle,rolling.status.enabled_status,rolling.status.estimate_status\
,gga_timestamp\
,gga_llh.latitude,gga_llh.longitude,gga_llh.altitude\
,gga_llh.gps_qual\
,eagleye_pp_llh.latitude,eagleye_pp_llh.longitude,eagleye_pp_llh.altitude\
,eagleye_pp_llh.orientation_covariance[0],eagleye_pp_llh.orientation_covariance[1],eagleye_pp_llh.orientation_covariance[2],eagleye_pp_llh.orientation_covariance[3],eagleye_pp_llh.orientation_covariance[4],eagleye_pp_llh.orientation_covariance[5],eagleye_pp_llh.orientation_covariance[6],eagleye_pp_llh.orientation_covariance[7],eagleye_pp_llh.orientation_covariance[8]\
,eagleye_pp_llh.status\
,eagleye_pp_llh.height_status\
,enu_relative_pos.enu_pos.x,enu_relative_pos.enu_pos.y,enu_relative_pos.enu_pos.z\
,enu_relative_pos.status.enabled_status\
" << std::endl;
  _log_header_make = true;
  }
  else
  {
  std::ofstream output_log_file(output_log_dir, std::ios_base::app);
  output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << _imu.header.stamp.toNSec() << ","; // timestamp
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.angular_velocity.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.angular_velocity.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.angular_velocity.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.linear_acceleration.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.linear_acceleration.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _imu.linear_acceleration.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << _rtklib_nav.tow << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_pos.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_pos.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_pos.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_vel.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_vel.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.ecef_vel.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(_rtklib_nav.status.status.status) << ",";
  output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << _rtklib_nav.status.status.service << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.status.latitude << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.status.longitude << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rtklib_nav.status.altitude << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.linear.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.linear.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.linear.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.angular.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.angular.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity.twist.angular.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _velocity_scale_factor.scale_factor << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.linear.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.linear.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.linear.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.angular.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.angular.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _correction_velocity.twist.angular.z << ",";
  output_log_file << (_velocity_scale_factor.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_velocity_scale_factor.status.estimate_status ? "1" : "0") << ",";
  if(_distance_ptr != nullptr)
  {
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _distance_ptr->distance << ",";
    output_log_file << (_distance_ptr->status.enabled_status ? "1" : "0") << ",";
    output_log_file << (_distance_ptr->status.estimate_status ? "1" : "0") << ",";
  } else
  {
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ",";
    output_log_file << 0 << ",";
    output_log_file << 0 << ",";
  }
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_1st.heading_angle << ",";
  output_log_file << (_heading_1st.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_heading_1st.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_interpolate_1st.heading_angle << ",";
  output_log_file << (_heading_interpolate_1st.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_heading_interpolate_1st.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_2nd.heading_angle << ",";
  output_log_file << (_heading_2nd.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_heading_2nd.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_interpolate_2nd.heading_angle << ",";
  output_log_file << (_heading_interpolate_2nd.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_heading_interpolate_2nd.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_3rd.heading_angle << ",";
  output_log_file << (_heading_3rd.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_heading_3rd.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _heading_interpolate_3rd.heading_angle << ",";
  output_log_file << (_heading_interpolate_3rd.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_heading_interpolate_3rd.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _yaw_rate_offset_stop.yaw_rate_offset << ",";
  output_log_file << (_yaw_rate_offset_stop.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_yaw_rate_offset_stop.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _yaw_rate_offset_1st.yaw_rate_offset << ",";
  output_log_file << (_yaw_rate_offset_1st.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_yaw_rate_offset_1st.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _yaw_rate_offset_2nd.yaw_rate_offset << ",";
  output_log_file << (_yaw_rate_offset_2nd.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_yaw_rate_offset_2nd.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _slip_angle.coefficient << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _slip_angle.slip_angle << ",";
  output_log_file << (_slip_angle.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_slip_angle.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_vel.vector.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_vel.vector.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_vel.vector.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.enu_pos.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.enu_pos.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.enu_pos.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.ecef_base_pos.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.ecef_base_pos.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos.ecef_base_pos.z << ",";
  output_log_file << (_enu_absolute_pos.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_enu_absolute_pos.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.enu_pos.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.enu_pos.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.enu_pos.z << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.ecef_base_pos.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.ecef_base_pos.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_absolute_pos_interpolate.ecef_base_pos.z << ",";
  output_log_file << (_enu_absolute_pos_interpolate.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_enu_absolute_pos_interpolate.status.estimate_status ? "1" : "0") << ",";
  // output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << angular_velocity_offset_stop.rollrate_offset << ",";
  // output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << angular_velocity_offset_stop.pitchrate_offset << ",";
  // output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << angular_velocity_offset_stop.yaw_rate_offset << ",";
  // output_log_file << (angular_velocity_offset_stop.status.enabled_status ? "1" : "0") << ",";
  // output_log_file << (angular_velocity_offset_stop.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _height.height << ",";
  output_log_file << (_height.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_height.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _pitching.pitching_angle << ",";
  output_log_file << (_pitching.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_pitching.status.estimate_status ? "1" : "0") << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // acc_x_offset
  output_log_file << 0 << ","; // acc_x_offset.status.enabled_status
  output_log_file << 0 << ","; // acc_x_offset.status.estimate_status
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; // acc_x_scale_factor.acc_x_scale_factor
  output_log_file << 0 << ","; // acc_x_scale_factor.status.enabled_status
  output_log_file << 0 << ","; // acc_x_scale_factor.status.estimate_status
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _rolling.rolling_angle << ",";
  output_log_file << (_rolling.status.enabled_status ? "1" : "0") << ",";
  output_log_file << (_rolling.status.estimate_status ? "1" : "0") << ",";
  if(_gga_ptr != nullptr)
  {
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << _gga_ptr->header.stamp.toNSec() << ","; //timestamp
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _gga_ptr->lat << ","; //gga_llh.latitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _gga_ptr->lon << ","; //gga_llh.longitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _gga_ptr->alt +  _gga_ptr->undulation<< ","; //gga_llh.altitude
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(_gga_ptr->gps_qual) << ","; //gga_llh.gps_qual
  } else
  {
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //timestamp
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //gga_llh.latitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //gga_llh.longitude
    output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0<< ","; //gga_llh.altitude
    output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //gga_llh.gps_qual
  }
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _eagleye_fix.latitude << ","; //eagleye_pp_llh.latitude
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _eagleye_fix.longitude << ","; //eagleye_pp_llh.longitude
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _eagleye_fix.altitude << ","; //eagleye_pp_llh.altitude
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[0]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[1]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[2]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[3]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[4]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[5]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[6]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[7]
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[8]
  output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //eagleye_pp_llh.status
  output_log_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //eagleye_pp_llh.status
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_relative_pos.enu_pos.x << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_relative_pos.enu_pos.y << ",";
  output_log_file << std::setprecision(std::numeric_limits<double>::max_digits10) << _enu_relative_pos.enu_pos.z << ",";
  output_log_file << (_enu_relative_pos.status.enabled_status ? "1" : "0");
  output_log_file << "\n";
  }
  return;
}

void timer_callback(const ros::TimerEvent& e, diagnostic_updater::Updater * updater)
{
  // Diagnostic Updater
  updater->force_update();

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu = *msg;

  if(_print_status)
  {
    printStatus();
  }

  if(_log_output_status)
  {
    outputLog();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "monitor");

  ros::NodeHandle n;

  std::string subscribe_twist_topic_name = "vehicle/twist";
  std::string subscribe_rtklib_nav_topic_name = "navsat/rtklib_nav";
  std::string subscribe_gga_topic_name = "navsat/gga";
  std::string comparison_twist_topic_name = "/calculated_twist";

  std::string yaml_file;
  n.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    n.getParam("use_rtk_dead_reckoning",_use_rtk_dead_reckoning);
    comparison_twist_topic_name = conf["monitor"]["comparison_twist_topic"].as<std::string>();

    _print_status = conf["monitor"]["print_status"].as<bool>();
    _log_output_status = conf["monitor"]["log_output_status"].as<bool>();
    _update_rate = conf["monitor"]["update_rate"].as<double>();
    _th_gnss_deadrock_time = conf["monitor"]["th_gnss_deadrock_time"].as<double>();
    _use_compare_yaw_rate = conf["monitor"]["use_compare_yaw_rate"].as<bool>();
    _th_diff_rad_per_sec = conf["monitor"]["th_diff_rad_per_sec"].as<double>();
    _th_num_continuous_abnormal_yaw_rate = conf["monitor"]["th_num_continuous_abnormal_yaw_rate"].as<double>();
    _th_dr_distance = conf["monitor"]["th_dr_distance"].as<double>();

    std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
    std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
    std::cout<< "subscribe_gga_topic_name "<<subscribe_gga_topic_name<<std::endl;
    std::cout<< "print_status "<<_print_status<<std::endl;
    std::cout<< "log_output_status "<<_log_output_status<<std::endl;
    std::cout<< "update_rate "<<_update_rate<<std::endl;
    std::cout<< "th_gnss_deadrock_time "<<_th_gnss_deadrock_time<<std::endl;
    std::cout<< "use_compare_yaw_rate "<<_use_compare_yaw_rate<<std::endl;
    std::cout<< "comparison_twist_topic_name "<<comparison_twist_topic_name<<std::endl;
    std::cout<< "th_diff_rad_per_sec "<<_th_diff_rad_per_sec<<std::endl;
    std::cout<< "use_rtk_dead_reckoning "<<_use_rtk_dead_reckoning<<std::endl;
    std::cout<< "th_dr_distance "<<_th_dr_distance<<std::endl;

  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mmonitor Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

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
  updater.add("eagleye_enu_vel", enu_vel_topic_checker);
  updater.add("eagleye_height", height_topic_checker);
  updater.add("eagleye_pitching", pitching_topic_checker);
  updater.add("eagleye_enu_absolute_pos", enu_absolute_pos_topic_checker);
  updater.add("eagleye_enu_absolute_pos_interpolate", enu_absolute_pos_interpolate_topic_checker);
  updater.add("eagleye_twist", twist_topic_checker);
  if(_use_compare_yaw_rate) updater.add("eagleye_corrected_imu", corrected_imu_topic_checker);
  if(_use_rtk_dead_reckoning) updater.add("eagleye_dead_reckoning_distance", dr_distance_checker);

  time_t time_;
  time_ = time(NULL);
  std::stringstream time_ss;
  time_ss << time_;
  std::string time_str = time_ss.str();
  output_log_dir = ros::package::getPath("eagleye_rt") + "/log/eagleye_log_" + time_str + ".csv";

  if(_log_output_status) std::cout << output_log_dir << std::endl;

  ros::Subscriber sub1 = n.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("rtklib/fix", 1000, rtklib_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_gga_topic_name, 1000, navsatfix_gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("velocity", 1000, correction_velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub8 = n.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub9 = n.subscribe("heading_1st", 1000, heading_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub10 = n.subscribe("heading_interpolate_1st", 1000, heading_interpolate_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub11 = n.subscribe("heading_2nd", 1000, heading_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub12 = n.subscribe("heading_interpolate_2nd", 1000, heading_interpolate_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub13 = n.subscribe("heading_3rd", 1000, heading_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub14 = n.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub15 = n.subscribe("yaw_rate_offset_stop", 1000, yaw_rate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub16 = n.subscribe("yaw_rate_offset_1st", 1000, yaw_rate_offset_1st_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub17 = n.subscribe("yaw_rate_offset_2nd", 1000, yaw_rate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub18 = n.subscribe("slip_angle", 1000, slip_angle_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub19 = n.subscribe("enu_relative_pos", 1000, enu_relative_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub20 = n.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub21 = n.subscribe("height", 1000, height_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub22 = n.subscribe("pitching", 1000, pitching_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub23 = n.subscribe("enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub24 = n.subscribe("enu_absolute_pos_interpolate", 1000, enu_absolute_pos_interpolate_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub25 = n.subscribe("fix", 1000, eagleye_fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub26 = n.subscribe("twist", 1000, eagleye_twist_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub27 = n.subscribe(comparison_twist_topic_name, 1000, comparison_velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub28 = n.subscribe("imu/data_corrected", 1000, corrected_imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub29 = n.subscribe("rolling", 1000, rolling_callback, ros::TransportHints().tcpNoDelay());

  ros::Timer timer = n.createTimer(ros::Duration(1/_update_rate), boost::bind(timer_callback,_1, &updater));

  ros::spin();

  return 0;
}