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

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "rtklib_msgs/RtklibNav.h"
#include "eagleye_msgs/Distance.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/Position.h"
#include "eagleye_msgs/SlipAngle.h"
#include <boost/circular_buffer.hpp>


#ifndef NAVIGATION_H
#define NAVIGATION_H

struct VelocityScaleFactorParam
{
  double estimated_number_min;
  double estimated_number_max;
  double estimated_velocity_threshold;
  double estimated_coefficient;
};

struct VelocityScaleFactorStatus
{
  std::vector<bool> gnss_status_buffer;
  std::vector<double> doppler_velocity_buffer;
  std::vector<double> velocity_buffer;
  int tow_last, estimated_number;
  double velocity_scale_factor_last;
  bool estimate_start_status;
};

struct YawrateOffsetStopParam
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double estimated_number;
};

struct YawrateOffsetStopStatus
{
  int stop_count;
  double yawrate_offset_stop_last;
  bool estimate_start_status;
  std::vector<double> yawrate_buffer;
};

struct YawrateOffsetParam
{
  bool reverse_imu;
  double estimated_number_min;
  double estimated_number_max;
  double estimated_coefficient;
  double estimated_velocity_threshold;
};

struct YawrateOffsetStatus
{
  bool estimate_start_status;
  int estimated_preparation_conditions;
  int heading_estimate_status_count;
  int estimated_number;
  std::vector<double> time_buffer;
  std::vector<double> yawrate_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<bool> heading_estimate_status_buffer;
  std::vector<double> yawrate_offset_stop_buffer;
};

struct HeadingParam
{
  bool reverse_imu;
  double estimated_number_min;
  double estimated_number_max;
  double estimated_gnss_coefficient;
  double estimated_heading_coefficient;
  double outlier_threshold;
  double estimated_velocity_threshold;
  double stop_judgment_velocity_threshold;
  double estimated_yawrate_threshold;

};

struct HeadingStatus
{
  int tow_last;
  int estimated_number;
  std::vector<double> time_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> yawrate_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> yawrate_offset_stop_buffer;
  std::vector<double> yawrate_offset_buffer;
  std::vector<double> slip_angle_buffer;
  std::vector<double> gnss_status_buffer;
};

struct HeadingInterpolateParam
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double number_buffer_max;
};

struct HeadingInterpolateStatus
{
  int number_buffer;
  int heading_estimate_status_count;
  double heading_stamp_last;
  double time_last;
  double provisional_heading_angle;
  std::vector<double> provisional_heading_angle_buffer;
  std::vector<double> imu_stamp_buffer;
};

struct PositionParam
{
  double estimated_distance;
  double separation_distance;
  double estimated_velocity_threshold;
  double outlier_threshold;
  double estimated_enu_vel_coefficient;
  double estimated_position_coefficient;
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
};

struct PositionStatus
{
  int estimated_number;
  int tow_last;
  int heading_estimate_status_count;
  double time_last;
  double enu_relative_pos_x, enu_relative_pos_y, enu_relative_pos_z;
  double distance_last;
  std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
  std::vector<double> enu_relative_pos_x_buffer, enu_relative_pos_y_buffer, enu_relative_pos_z_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> distance_buffer;
};

struct PositionInterpolateParam
{
  double number_buffer_max;
  bool altitude_estimate;
};

struct PositionInterpolateStatus
{
  int position_estimate_status_count;
  int number_buffer;
  double position_stamp_last;
  double time_last;
  double provisional_enu_pos_x;
  double provisional_enu_pos_y;
  double provisional_enu_pos_z;
  std::vector<double> provisional_enu_pos_x_buffer;
  std::vector<double> provisional_enu_pos_y_buffer;
  std::vector<double> provisional_enu_pos_z_buffer;
  std::vector<double> imu_stamp_buffer;
};

struct SlipangleParam
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double manual_coefficient;
};

struct SmoothingStatus
{
  double last_pos[3];
  std::vector<double> time_buffer;
  std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
  std::vector<double> correction_velocity_buffer;
};

struct TrajectoryParam
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
};

struct TrajectoryStatus
{
  int estimate_status_count;
  double time_last;
};

extern void calc_velocity_scale_factor(const rtklib_msgs::RtklibNav, const geometry_msgs::TwistStamped, const VelocityScaleFactorParam, VelocityScaleFactorStatus*, eagleye_msgs::VelocityScaleFactor*);
extern void calc_yawrate_offset_stop(const geometry_msgs::TwistStamped, const sensor_msgs::Imu, const YawrateOffsetStopParam, YawrateOffsetStopStatus*, eagleye_msgs::YawrateOffset*);
extern void calc_yawrate_offset(const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const sensor_msgs::Imu, const YawrateOffsetParam, YawrateOffsetStatus*, eagleye_msgs::YawrateOffset*);
extern void calc_heading(const rtklib_msgs::RtklibNav, const sensor_msgs::Imu, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::YawrateOffset, const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const HeadingParam, HeadingStatus*,eagleye_msgs::Heading*);
extern void calc_position(const rtklib_msgs::RtklibNav, const eagleye_msgs::Position, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::Distance, const eagleye_msgs::Heading, const geometry_msgs::Vector3Stamped, const PositionParam, PositionStatus*, eagleye_msgs::Position*);
extern void calc_slip_angle(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const SlipangleParam,eagleye_msgs::SlipAngle*);
extern void calc_smoothing(const rtklib_msgs::RtklibNav,const eagleye_msgs::VelocityScaleFactor,const PositionParam,SmoothingStatus*,eagleye_msgs::Position*);
extern void calc_trajectory(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::Heading,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const TrajectoryParam,TrajectoryStatus*,geometry_msgs::Vector3Stamped*,eagleye_msgs::Position*,geometry_msgs::TwistStamped*);
extern void calc_heading_interpolate(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const eagleye_msgs::SlipAngle,const HeadingInterpolateParam,HeadingInterpolateStatus*,eagleye_msgs::Heading*);
extern void calc_position_interpolate(const eagleye_msgs::Position,const geometry_msgs::Vector3Stamped,const PositionInterpolateParam,PositionInterpolateStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);


#endif /*NAVIGATION_H */
