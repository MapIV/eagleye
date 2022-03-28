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
#include "nmea_msgs/Gpgga.h"
#include "nmea_msgs/Gprmc.h"

#include "rtklib_msgs/RtklibNav.h"

#include "eagleye_msgs/Distance.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/Position.h"
#include "eagleye_msgs/SlipAngle.h"
#include "eagleye_msgs/AccXScaleFactor.h"
#include "eagleye_msgs/AccXOffset.h"
#include "eagleye_msgs/Height.h"
#include "eagleye_msgs/Pitching.h"
#include "eagleye_msgs/AngularVelocityOffset.h"
#include "eagleye_msgs/Rolling.h"
#include "eagleye_msgs/AccYOffset.h"
#include <boost/circular_buffer.hpp>
#include <math.h>
#include <numeric>

#ifndef NAVIGATION_H
#define NAVIGATION_H

struct VelocityScaleFactorParameter
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
  double rmc_time_last;
  double velocity_scale_factor_last;
  bool estimate_start_status;
};

struct DistanceStatus
{
  double time_last;
};

struct YawrateOffsetStopParameter
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double estimated_number;
  double outlier_threshold;
};

struct YawrateOffsetStopStatus
{
  int stop_count;
  double yawrate_offset_stop_last;
  bool estimate_start_status;
  std::vector<double> yawrate_buffer;
};

struct YawrateOffsetParameter
{
  bool reverse_imu;
  double estimated_number_min;
  double estimated_number_max;
  double estimated_coefficient;
  double estimated_velocity_threshold;
  double outlier_threshold;
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

struct HeadingParameter
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
  double rmc_time_last;
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

struct RtkHeadingParameter
{
  bool reverse_imu;
  double estimated_distance;
  double estimated_heading_buffer_min;
  double estimated_number_min;
  double estimated_number_max;
  double estimated_gnss_coefficient;
  double estimated_heading_coefficient;
  double outlier_threshold;
  double estimated_velocity_threshold;
  double stop_judgment_velocity_threshold;
  double estimated_yawrate_threshold;

};

struct RtkHeadingStatus
{
  int tow_last;
  int estimated_number;
  double last_rtk_heading_angle;
  std::vector<double> time_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> yawrate_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> yawrate_offset_stop_buffer;
  std::vector<double> yawrate_offset_buffer;
  std::vector<double> slip_angle_buffer;
  std::vector<double> gnss_status_buffer;
  std::vector<double> distance_buffer;
  std::vector<double> latitude_buffer;
  std::vector<double> longitude_buffer;
  std::vector<double> altitude_buffer;
  std::vector<int> fix_status_buffer;
};

struct HeadingInterpolateParameter
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double number_buffer_max;
};

struct HeadingInterpolateStatus
{
  int number_buffer;
  int heading_estimate_status_count;
  bool heading_estimate_start_status;
  double heading_stamp_last;
  double time_last;
  double provisional_heading_angle;
  std::vector<double> provisional_heading_angle_buffer;
  std::vector<double> imu_stamp_buffer;
};

struct PositionParameter
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
  double tf_gnss_translation_x;
  double tf_gnss_translation_y;
  double tf_gnss_translation_z;
  double tf_gnss_rotation_x;
  double tf_gnss_rotation_y;
  double tf_gnss_rotation_z;
  double tf_gnss_rotation_w;
  std::string tf_gnss_parent_flame;
  std::string tf_gnss_child_flame;
};

struct PositionStatus
{
  int estimated_number;
  int tow_last;
  int heading_estimate_status_count;
  double nmea_time_last;
  double time_last;
  double enu_relative_pos_x, enu_relative_pos_y, enu_relative_pos_z;
  double distance_last;
  double enu_pos[3];
  bool gnss_update_failure;
  std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
  std::vector<double> enu_relative_pos_x_buffer, enu_relative_pos_y_buffer, enu_relative_pos_z_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> distance_buffer;
};

struct PositionInterpolateParameter
{
  double number_buffer_max;
  double stop_judgment_velocity_threshold;
};

struct PositionInterpolateStatus
{
  int position_estimate_status_count;
  int number_buffer;
  bool position_estimate_start_status;
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

struct SlipangleParameter
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double manual_coefficient;
};

struct SlipCoefficientParameter
{
  bool reverse_imu;
  double estimated_number_min;
  double estimated_number_max;
  double estimated_velocity_threshold;
  double estimated_yawrate_threshold;
  double lever_arm;
  double stop_judgment_velocity_threshold;
};

struct SlipCoefficientStatus
{
  double heading_estimate_status_count;
  std::vector<double> doppler_slip_buffer;
  std::vector<double> acceleration_y_buffer;
};

struct SmoothingParameter
{
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
  int estimated_number_max;
  double estimated_velocity_threshold;
  double estimated_threshold;
};

struct SmoothingStatus
{
  int estimated_number;
  double last_pos[3];
  std::vector<double> time_buffer;
  std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
  std::vector<double> correction_velocity_buffer;
};

struct TrajectoryParameter
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double stop_judgment_yawrate_threshold;
};

struct TrajectoryStatus
{
  int estimate_status_count;
  double heading_last;
  double time_last;
};

struct HeightParameter
{
  double estimated_distance;
  double estimated_distance_max;
  double separation_distance;
  double estimated_velocity_threshold;
  double estimated_velocity_coefficient;
  double estimated_height_coefficient;
  double outlier_threshold;
  int average_num;
};

struct HeightStatus
{
  double relative_height_G;
  double relative_height_diffvel;
  double relative_height_offset;
  double acceleration_offset_linear_x_last;
  double acceleration_SF_linear_x_last;
  double height_last;
  double time_last;
  double distance_last;
  double correction_velocity_x_last;
  double fix_time_last;
  double pitching_angle_last;
  bool height_estimate_start_status;
  bool estimate_start_status;
  bool acceleration_SF_estimate_status;
  int data_number;
  bool flag_reliability;
  std::vector<double> height_buffer;
  std::vector<double> height_buffer2;
  std::vector<double> relative_height_G_buffer;
  std::vector<double> relative_height_diffvel_buffer;
  std::vector<double> relative_height_offset_buffer;
  std::vector<double> correction_relative_height_buffer;
  std::vector<double> correction_relative_height_buffer2;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> distance_buffer;
  std::vector<double> acc_buffer;
};

struct AngularVelocityOffsetStopParameter
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double estimated_number;
  double outlier_threshold;
};

struct AngularVelocityOffsetStopStatus
{
  int stop_count;
  double rollrate_offset_stop_last;
  double pitchrate_offset_stop_last;
  double yawrate_offset_stop_last;
  bool estimate_start_status;
  std::vector<double> rollrate_buffer;
  std::vector<double> pitchrate_buffer;
  std::vector<double> yawrate_buffer;
};

struct RtkDeadreckoningParameter
{
  double stop_judgment_velocity_threshold;
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
  bool use_ecef_base_position;
  double tf_gnss_translation_x;
  double tf_gnss_translation_y;
  double tf_gnss_translation_z;
  double tf_gnss_rotation_x;
  double tf_gnss_rotation_y;
  double tf_gnss_rotation_z;
  double tf_gnss_rotation_w;
  std::string tf_gnss_parent_flame;
  std::string tf_gnss_child_flame;
};

struct RtkDeadreckoningStatus
{
  int position_estimate_status_count;
  int number_buffer;
  bool position_estimate_start_status;
  bool ecef_base_pos_status;
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

struct EnableAdditionalRollingParameter
{
  bool reverse_imu;
  bool reverse_imu_angular_velocity_x;
  bool reverse_imu_linear_acceleration_y;
  double matching_update_distance;
  double stop_judgment_velocity_threshold;
  double rolling_buffer_num;
  double link_Time_stamp_parameter;
  double imu_buffer_num;
};

struct EnableAdditionalRollingStatus
{
  double distance_last;
  double acc_offset_sum;
  double yawrate;
  double rollrate;
  double imu_acceleration_y;
  double rollrate_offset_stop;
  double imu_time_last;
  double localization_time_last;
  int acc_offset_data_count;
  std::vector<double> roll_rate_interpolate_buffer;
  std::vector<double> rolling_estimated_buffer;
  std::vector<double> imu_time_buffer;
  std::vector<double> yawrate_buffer;
  std::vector<double> velocity_buffer;
  std::vector<double> yawrate_offset_buffer;
  std::vector<double> acceleration_y_buffer;
  std::vector<double> distance_buffer;
};

struct RollingParameter
{
  bool reverse_imu;
  double stop_judgment_velocity_threshold;
  double filter_process_noise;
  double filter_observation_noise;
};

struct RollingStatus
{
  double acceleration_y_last;
  double acceleration_y_variance_last;
  double rolling_last;
  bool data_status;
};

extern void velocity_scale_factor_estimate(const rtklib_msgs::RtklibNav, const geometry_msgs::TwistStamped, const VelocityScaleFactorParameter, VelocityScaleFactorStatus*, eagleye_msgs::VelocityScaleFactor*);
extern void velocity_scale_factor_estimate(const nmea_msgs::Gprmc, const geometry_msgs::TwistStamped, const VelocityScaleFactorParameter, VelocityScaleFactorStatus*, eagleye_msgs::VelocityScaleFactor*);
extern void distance_estimate(const eagleye_msgs::VelocityScaleFactor, DistanceStatus*,eagleye_msgs::Distance*);
extern void yawrate_offset_stop_estimate(const geometry_msgs::TwistStamped, const sensor_msgs::Imu, const YawrateOffsetStopParameter, YawrateOffsetStopStatus*, eagleye_msgs::YawrateOffset*);
extern void yawrate_offset_estimate(const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const sensor_msgs::Imu, const YawrateOffsetParameter, YawrateOffsetStatus*, eagleye_msgs::YawrateOffset*);
extern void heading_estimate(const rtklib_msgs::RtklibNav, const sensor_msgs::Imu, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::YawrateOffset, const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const HeadingParameter, HeadingStatus*,eagleye_msgs::Heading*);
extern void heading_estimate(const nmea_msgs::Gprmc, const sensor_msgs::Imu, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::YawrateOffset, const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const HeadingParameter, HeadingStatus*,eagleye_msgs::Heading*);
extern void position_estimate(const rtklib_msgs::RtklibNav, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::Distance, const eagleye_msgs::Heading, const geometry_msgs::Vector3Stamped, const PositionParameter, PositionStatus*, eagleye_msgs::Position*);
extern void position_estimate(const sensor_msgs::NavSatFix, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::Distance, const eagleye_msgs::Heading, const geometry_msgs::Vector3Stamped, const PositionParameter, PositionStatus*, eagleye_msgs::Position*);
extern void slip_angle_estimate(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const SlipangleParameter,eagleye_msgs::SlipAngle*);
extern void slip_coefficient_estimate(const sensor_msgs::Imu,const rtklib_msgs::RtklibNav,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const SlipCoefficientParameter,SlipCoefficientStatus*,double*);
extern void smoothing_estimate(const rtklib_msgs::RtklibNav,const eagleye_msgs::VelocityScaleFactor,const SmoothingParameter,SmoothingStatus*,eagleye_msgs::Position*);
extern void trajectory_estimate(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::Heading,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const TrajectoryParameter,TrajectoryStatus*,geometry_msgs::Vector3Stamped*,eagleye_msgs::Position*,geometry_msgs::TwistStamped*);
extern void heading_interpolate_estimate(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const eagleye_msgs::SlipAngle,const HeadingInterpolateParameter,HeadingInterpolateStatus*,eagleye_msgs::Heading*);
extern void position_interpolate_estimate(const eagleye_msgs::Position,const geometry_msgs::Vector3Stamped,const eagleye_msgs::Position,const eagleye_msgs::Height,const PositionInterpolateParameter,PositionInterpolateStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);
extern void pitching_estimate(const sensor_msgs::Imu,const sensor_msgs::NavSatFix,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::Distance,const HeightParameter,HeightStatus*,eagleye_msgs::Height*,eagleye_msgs::Pitching*,eagleye_msgs::AccXOffset*,eagleye_msgs::AccXScaleFactor*);
extern void trajectory3d_estimate(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::Heading,const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Pitching,const TrajectoryParameter,TrajectoryStatus*,geometry_msgs::Vector3Stamped*,eagleye_msgs::Position*,geometry_msgs::TwistStamped*);
extern void angular_velocity_offset_stop_estimate(const geometry_msgs::TwistStamped, const sensor_msgs::Imu, const AngularVelocityOffsetStopParameter, AngularVelocityOffsetStopStatus*, eagleye_msgs::AngularVelocityOffset*);
extern void rtk_deadreckoning_estimate(const rtklib_msgs::RtklibNav,const geometry_msgs::Vector3Stamped,const sensor_msgs::NavSatFix, const eagleye_msgs::Heading,const RtkDeadreckoningParameter,RtkDeadreckoningStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);
extern void rtk_deadreckoning_estimate(const geometry_msgs::Vector3Stamped,const sensor_msgs::NavSatFix, const eagleye_msgs::Heading,const RtkDeadreckoningParameter,RtkDeadreckoningStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);
extern void rtk_heading_estimate(const sensor_msgs::NavSatFix, const sensor_msgs::Imu, const eagleye_msgs::VelocityScaleFactor, const eagleye_msgs::Distance,const eagleye_msgs::YawrateOffset, const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const RtkHeadingParameter, RtkHeadingStatus*,eagleye_msgs::Heading*);
extern void enable_additional_rolling_estimate(const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset ,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Distance,const sensor_msgs::Imu,const geometry_msgs::PoseStamped,const eagleye_msgs::AngularVelocityOffset,const EnableAdditionalRollingParameter,EnableAdditionalRollingStatus*,eagleye_msgs::Rolling*,eagleye_msgs::AccYOffset*);
extern void rolling_estimate(const sensor_msgs::Imu,const eagleye_msgs::VelocityScaleFactor,const eagleye_msgs::YawrateOffset ,const eagleye_msgs::YawrateOffset,const RollingParameter,RollingStatus*,eagleye_msgs::Rolling*);

#endif /*NAVIGATION_H */
