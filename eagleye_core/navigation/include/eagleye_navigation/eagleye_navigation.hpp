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

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nmea_msgs/msg/gpgga.hpp"
#include "nmea_msgs/msg/gprmc.hpp"

#include "rtklib_msgs/msg/rtklib_nav.hpp"

#include "eagleye_msgs/msg/status.hpp"
#include "eagleye_msgs/msg/status_stamped.hpp"
#include "eagleye_msgs/msg/distance.hpp"
#include "eagleye_msgs/msg/yawrate_offset.hpp"
#include "eagleye_msgs/msg/velocity_scale_factor.hpp"
#include "eagleye_msgs/msg/heading.hpp"
#include "eagleye_msgs/msg/position.hpp"
#include "eagleye_msgs/msg/slip_angle.hpp"
#include "eagleye_msgs/msg/acc_x_scale_factor.hpp"
#include "eagleye_msgs/msg/acc_x_offset.hpp"
#include "eagleye_msgs/msg/height.hpp"
#include "eagleye_msgs/msg/pitching.hpp"
#include "eagleye_msgs/msg/angular_velocity_offset.hpp"
#include "eagleye_msgs/msg/acc_y_offset.hpp"
#include "eagleye_msgs/msg/rolling.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <math.h>
#include <numeric>

#include <rclcpp/rclcpp.hpp>

#ifndef NAVIGATION_H
#define NAVIGATION_H

struct VelocityScaleFactorParameter
{
  double imu_rate;
  double gnss_rate;
  double moving_judgment_threshold;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  bool save_velocity_scale_factor{false};
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
  double imu_rate;
  double estimated_interval;
  double stop_judgment_threshold;
  double outlier_threshold;
};

struct YawrateOffsetStopStatus
{
  int stop_count;
  double yaw_rate_offset_stop_last;
  bool estimate_start_status;
  std::vector<double> yaw_rate_buffer;
};

struct YawrateOffsetParameter
{
  double imu_rate;
  double gnss_rate;
  double moving_judgment_threshold;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  double outlier_threshold;
};


struct YawrateOffsetStatus
{
  bool estimate_start_status;
  int estimated_preparation_conditions;
  int heading_estimate_status_count;
  int estimated_number;
  std::vector<double> time_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<bool> heading_estimate_status_buffer;
  std::vector<double> yaw_rate_offset_stop_buffer;
};

struct HeadingParameter
{
  double imu_rate;
  double gnss_rate;
  double stop_judgment_threshold;
  double moving_judgment_threshold;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double curve_judgment_threshold;
  double init_STD;
};

struct HeadingStatus
{
  int tow_last;
  double rmc_time_last;
  double ros_time_last;
  int estimated_number;
  std::vector<double> time_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> yaw_rate_offset_stop_buffer;
  std::vector<double> yaw_rate_offset_buffer;
  std::vector<double> slip_angle_buffer;
  std::vector<double> gnss_status_buffer;
};

struct RtkHeadingParameter
{
  double imu_rate;
  double gnss_rate;
  double stop_judgment_threshold;
  double slow_judgment_threshold;
  double update_distance;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double curve_judgment_threshold;
};

struct RtkHeadingStatus
{
  int tow_last;
  double rmc_time_last;
  int estimated_number;
  double last_rtk_heading_angle;
  std::vector<double> time_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> yaw_rate_offset_stop_buffer;
  std::vector<double> yaw_rate_offset_buffer;
  std::vector<double> slip_angle_buffer;
  std::vector<double> gnss_status_buffer;
  std::vector<double> distance_buffer;
  std::vector<double> latitude_buffer;
  std::vector<double> longitude_buffer;
  std::vector<double> altitude_buffer;
  std::vector<int> gga_status_buffer;
};

struct HeadingInterpolateParameter
{
  double imu_rate;
  double stop_judgment_threshold;
  double sync_search_period;
  double proc_noise;
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
  double heading_variance_last;
};

struct PositionParameter
{
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
  double tf_gnss_translation_x;
  double tf_gnss_translation_y;
  double tf_gnss_translation_z;
  double tf_gnss_rotation_x;
  double tf_gnss_rotation_y;
  double tf_gnss_rotation_z;
  double tf_gnss_rotation_w = 1.0;
  std::string tf_gnss_parent_frame;
  std::string tf_gnss_child_frame;

  double imu_rate;
  double gnss_rate;
  double moving_judgment_threshold;
  double estimated_interval;
  double update_distance;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
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
  double imu_rate;
  double stop_judgment_threshold;
  double sync_search_period;
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
  double manual_coefficient;
  double stop_judgment_threshold;
};

struct SlipCoefficientParameter
{
  double imu_rate;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double stop_judgment_threshold;
  double moving_judgment_threshold;
  double curve_judgment_threshold;
  double lever_arm;
};

struct SlipCoefficientStatus
{
  double heading_estimate_status_count;
  std::vector<double> doppler_slip_buffer;
  std::vector<double> acceleration_y_buffer;
};

struct SmoothingParameter
{
  double gnss_rate;
  double moving_judgment_threshold;
  double moving_average_time;
  double moving_ratio_threshold;
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
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
  double stop_judgment_threshold;
  double curve_judgment_threshold;
  double sensor_noise_velocity;
  double sensor_scale_noise_velocity;
  double sensor_noise_yaw_rate;
  double sensor_bias_noise_yaw_rate;
};

struct TrajectoryStatus
{
  int estimate_status_count;
  double heading_last;
  double time_last;
};

struct HeightParameter
{
  double imu_rate;
  double gnss_rate;
  double moving_judgment_threshold;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double update_distance;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double moving_average_time;
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
  double gga_time_last;
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
  double imu_rate;
  double estimated_interval;
  double stop_judgment_threshold;
  double outlier_threshold;
};

struct AngularVelocityOffsetStopStatus
{
  int stop_count;
  double rollrate_offset_stop_last;
  double pitch_rate_offset_stop_last;
  double yaw_rate_offset_stop_last;
  bool estimate_start_status;
  std::vector<double> rollrate_buffer;
  std::vector<double> pitch_rate_buffer;
  std::vector<double> yaw_rate_buffer;
};

struct RtkDeadreckoningParameter
{
  double stop_judgment_threshold;
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
  std::string tf_gnss_parent_frame;
  std::string tf_gnss_child_frame;
  double rtk_fix_STD;
  double proc_noise;
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
  Eigen::MatrixXd position_covariance_last;
};

struct EnableAdditionalRollingParameter
{
  double imu_rate;
  double stop_judgment_threshold;
  double update_distance;
  double moving_average_time;
  double sync_judgment_threshold;
  double sync_search_period;
};

struct EnableAdditionalRollingStatus
{
  double distance_last;
  double acc_offset_sum;
  double yaw_rate;
  double rollrate;
  double imu_acceleration_y;
  double rollrate_offset_stop;
  double imu_time_last;
  double localization_time_last;
  int acc_offset_data_count;
  std::vector<double> roll_rate_interpolate_buffer;
  std::vector<double> rolling_estimated_buffer;
  std::vector<double> imu_time_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> velocity_buffer;
  std::vector<double> yaw_rate_offset_buffer;
  std::vector<double> acceleration_y_buffer;
  std::vector<double> distance_buffer;
};

struct RollingParameter
{
  double stop_judgment_threshold;
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

extern void velocity_scale_factor_estimate(const rtklib_msgs::msg::RtklibNav, const geometry_msgs::msg::TwistStamped, const VelocityScaleFactorParameter,
  VelocityScaleFactorStatus*, geometry_msgs::msg::TwistStamped*, eagleye_msgs::msg::VelocityScaleFactor*);
extern void velocity_scale_factor_estimate(const nmea_msgs::msg::Gprmc, const geometry_msgs::msg::TwistStamped, const VelocityScaleFactorParameter,
  VelocityScaleFactorStatus*, geometry_msgs::msg::TwistStamped*, eagleye_msgs::msg::VelocityScaleFactor*);
extern void distance_estimate(const geometry_msgs::msg::TwistStamped, DistanceStatus*, eagleye_msgs::msg::Distance*);
extern void yaw_rate_offset_stop_estimate(const geometry_msgs::msg::TwistStamped, const sensor_msgs::msg::Imu, const YawrateOffsetStopParameter,
  YawrateOffsetStopStatus*, eagleye_msgs::msg::YawrateOffset*);
extern void yaw_rate_offset_estimate(const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::Heading,
  const sensor_msgs::msg::Imu, const YawrateOffsetParameter, YawrateOffsetStatus*, eagleye_msgs::msg::YawrateOffset*);
extern void heading_estimate(const rtklib_msgs::msg::RtklibNav, const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped,
  const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::YawrateOffset,  const eagleye_msgs::msg::SlipAngle, const eagleye_msgs::msg::Heading,
  const HeadingParameter, HeadingStatus*,eagleye_msgs::msg::Heading*);
extern void heading_estimate(const nmea_msgs::msg::Gprmc, const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped,
  const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::SlipAngle, const eagleye_msgs::msg::Heading,
  const HeadingParameter, HeadingStatus*,eagleye_msgs::msg::Heading*);
extern void heading_estimate(const eagleye_msgs::msg::Heading, const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::YawrateOffset,
  const eagleye_msgs::msg::YawrateOffset,  const eagleye_msgs::msg::SlipAngle, const eagleye_msgs::msg::Heading, const HeadingParameter, HeadingStatus*,eagleye_msgs::msg::Heading*);
extern void position_estimate(const rtklib_msgs::msg::RtklibNav, const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::StatusStamped,
  const eagleye_msgs::msg::Distance, const eagleye_msgs::msg::Heading, const geometry_msgs::msg::Vector3Stamped, const PositionParameter, PositionStatus*,
  eagleye_msgs::msg::Position*);
extern void position_estimate(const nmea_msgs::msg::Gpgga, const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::StatusStamped, const eagleye_msgs::msg::Distance,
  const eagleye_msgs::msg::Heading, const geometry_msgs::msg::Vector3Stamped, const PositionParameter, PositionStatus*, eagleye_msgs::msg::Position*);
extern void slip_angle_estimate(const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::StatusStamped,
  const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::YawrateOffset,const SlipangleParameter,eagleye_msgs::msg::SlipAngle*);
extern void slip_coefficient_estimate(const sensor_msgs::msg::Imu, const rtklib_msgs::msg::RtklibNav,const geometry_msgs::msg::TwistStamped,
  const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::Heading,const SlipCoefficientParameter,
  SlipCoefficientStatus*,double*);
extern void smoothing_estimate(const rtklib_msgs::msg::RtklibNav,const geometry_msgs::msg::TwistStamped,const SmoothingParameter,SmoothingStatus*,
  eagleye_msgs::msg::Position*);
extern void trajectory_estimate(const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::StatusStamped,
  const eagleye_msgs::msg::Heading, const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::YawrateOffset,const TrajectoryParameter,TrajectoryStatus*,
  geometry_msgs::msg::Vector3Stamped*,eagleye_msgs::msg::Position*,geometry_msgs::msg::TwistStamped*,geometry_msgs::msg::TwistWithCovarianceStamped*);
extern void heading_interpolate_estimate(const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped,
  const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::Heading, const eagleye_msgs::msg::SlipAngle,
  const HeadingInterpolateParameter,HeadingInterpolateStatus*, eagleye_msgs::msg::Heading*);
extern void position_interpolate_estimate(const eagleye_msgs::msg::Position, const geometry_msgs::msg::Vector3Stamped, const eagleye_msgs::msg::Position,
  const eagleye_msgs::msg::Height,const PositionInterpolateParameter, PositionInterpolateStatus*,eagleye_msgs::msg::Position*,sensor_msgs::msg::NavSatFix*);
extern void pitching_estimate(const sensor_msgs::msg::Imu, const nmea_msgs::msg::Gpgga, const geometry_msgs::msg::TwistStamped,
  const eagleye_msgs::msg::Distance,const HeightParameter,HeightStatus*,eagleye_msgs::msg::Height*,eagleye_msgs::msg::Pitching*,eagleye_msgs::msg::AccXOffset*,
  eagleye_msgs::msg::AccXScaleFactor*);
extern void trajectory3d_estimate(const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::StatusStamped, const eagleye_msgs::msg::Heading,
  const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::YawrateOffset,const eagleye_msgs::msg::Pitching,const TrajectoryParameter,TrajectoryStatus*,
  geometry_msgs::msg::Vector3Stamped*,eagleye_msgs::msg::Position*,geometry_msgs::msg::TwistStamped*,geometry_msgs::msg::TwistWithCovarianceStamped*);
extern void angular_velocity_offset_stop_estimate(const geometry_msgs::msg::TwistStamped, const sensor_msgs::msg::Imu,
  const AngularVelocityOffsetStopParameter, AngularVelocityOffsetStopStatus*, eagleye_msgs::msg::AngularVelocityOffset*);
extern void rtk_dead_reckoning_estimate(const rtklib_msgs::msg::RtklibNav,const geometry_msgs::msg::Vector3Stamped,const nmea_msgs::msg::Gpgga,
  const eagleye_msgs::msg::Heading,const RtkDeadreckoningParameter,RtkDeadreckoningStatus*,eagleye_msgs::msg::Position*,sensor_msgs::msg::NavSatFix*);
extern void rtk_dead_reckoning_estimate(const geometry_msgs::msg::Vector3Stamped,const nmea_msgs::msg::Gpgga, const eagleye_msgs::msg::Heading,
  const RtkDeadreckoningParameter,RtkDeadreckoningStatus*,eagleye_msgs::msg::Position*,sensor_msgs::msg::NavSatFix*);
extern void rtk_heading_estimate(const nmea_msgs::msg::Gpgga, const sensor_msgs::msg::Imu, const geometry_msgs::msg::TwistStamped,
  const eagleye_msgs::msg::Distance,const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::YawrateOffset,  const eagleye_msgs::msg::SlipAngle,
  const eagleye_msgs::msg::Heading, const RtkHeadingParameter, RtkHeadingStatus*,eagleye_msgs::msg::Heading*);
extern void enable_additional_rolling_estimate(const geometry_msgs::msg::TwistStamped, const eagleye_msgs::msg::StatusStamped,
  const eagleye_msgs::msg::YawrateOffset ,const eagleye_msgs::msg::YawrateOffset, const eagleye_msgs::msg::Distance,
  const sensor_msgs::msg::Imu,const geometry_msgs::msg::PoseStamped,const eagleye_msgs::msg::AngularVelocityOffset,
  const EnableAdditionalRollingParameter,EnableAdditionalRollingStatus*,eagleye_msgs::msg::Rolling*,eagleye_msgs::msg::AccYOffset*);
extern void rolling_estimate(const sensor_msgs::msg::Imu,const geometry_msgs::msg::TwistStamped,const eagleye_msgs::msg::YawrateOffset ,const eagleye_msgs::msg::YawrateOffset,
  const RollingParameter,RollingStatus*,eagleye_msgs::msg::Rolling*);

class VelocityEstimator
{
  public:
    VelocityEstimator();

    // Parameter setting
    void setParam(std::string yaml_file);

    // Main estimate function
    void VelocityEstimate(const sensor_msgs::msg::Imu, const rtklib_msgs::msg::RtklibNav, const nmea_msgs::msg::Gpgga, geometry_msgs::msg::TwistStamped*);

    eagleye_msgs::msg::Status getStatus();

  private:

    class PitchrateOffsetStopEstimator
    {
      public:
        PitchrateOffsetStopEstimator();

        void setParam(std::string yaml_file);
        bool PitchrateOffsetStopEstimate(double pitch_rate, double stop_status);

        double pitch_rate_offset;
        eagleye_msgs::msg::Status pitch_rate_offset_status;

      private:
        struct Param
        {
          int imu_rate;
          int estimated_interval;
          int buffer_count_max;
        };
        PitchrateOffsetStopEstimator::Param param;

        int stop_count;
        std::vector<double> pitch_rate_buffer;
    };

    class PitchingEstimator
    {
      public:
        PitchingEstimator();

        void setParam(std::string yaml_file);
        bool PitchingEstimate(double imu_time_last, double doppler_velocity, double rtkfix_velocity,
                              double pitch_rate, double pitch_rate_offset, double rtkfix_pitching,
                              bool navsat_update_status, bool stop_status);

        double pitching;
        eagleye_msgs::msg::Status pitching_status;

      private:
        struct Param
        {
          double imu_rate;
          double gnss_rate;
          double estimated_interval;
          double buffer_max;
          double outlier_threshold;
          double estimated_velocity_threshold;
          double slow_judgment_threshold;
          double gnss_receiving_threshold;
          double estimated_gnss_coefficient;
          double outlier_ratio_threshold;
          double estimated_coefficient;
        };
        PitchingEstimator::Param param;

        std::vector<double> time_buffer;
        std::vector<double> corrected_pitch_rate_buffer;
        std::vector<double> rtkfix_pitching_buffer;
        std::vector<bool> use_gnss_status_buffer;
        std::vector<bool> navsat_update_status_buffer;
    };

    class AccelerationOffsetEstimator
    {
      public:
        AccelerationOffsetEstimator();

        void setParam(std::string yaml_file);
        bool AccelerationOffsetEstimate(double imu_time_last, double rtkfix_velocity, double pitching,
                              double acceleration, bool navsat_update_status);

        double filtered_acceleration;
        double acceleration_offset;
        eagleye_msgs::msg::Status acceleration_offset_status;

      private:
        struct Param
        {
          double imu_rate;
          double gnss_rate;
          double estimated_minimum_interval;
          double estimated_maximum_interval;
          double buffer_min;
          double buffer_max;
          double filter_process_noise;
          double filter_observation_noise;
        };
        AccelerationOffsetEstimator::Param param;

        double acceleration_last;
        double acceleration_variance_last;
        std::vector<double> time_buffer;
        std::vector<double> pitching_buffer; 
        std::vector<double> filtered_acceleration_buffer;
        std::vector<double> rtkfix_velocity_buffer; 
        std::vector<double> navsat_update_status_buffer;
    };

    // Parameter
    struct Param
    {
      double ecef_base_pos_x;
      double ecef_base_pos_y;
      double ecef_base_pos_z;
      bool use_ecef_base_position;

      double imu_rate;
      double gnss_rate;

      double gga_downsample_time;
      double stop_judgment_interval;
      double stop_judgment_velocity_threshold;
      double stop_judgment_buffer_maxnum;
      double variance_threshold;

      // doppler fusion parameter
      double estimated_interval;
      double buffer_max;
      double gnss_receiving_threshold;
      double estimated_gnss_coefficient;
      double outlier_ratio_threshold;
      double estimated_coefficient;
      double outlier_threshold;
    };
    VelocityEstimator::Param param;

    // imu variables
    double acceleration;
    double pitch_rate;
    double imu_time_last;

    // rtklib_nav variables
    double doppler_velocity;
    double rtklib_nav_time_last;
    bool rtklib_update_status;

    // gga variables
    double ecef_base_position[3]; 
    bool ecef_base_position_status;
    double gga_time_last;
    double gga_position_enu_last[3]; 
    int gga_status_last;
    double rtkfix_velocity; 
    double rtkfix_pitching; 
    bool navsat_update_status;

    // stop judgment variables
    bool stop_status;
    std::vector<double> angular_velocity_x_buffer;
    std::vector<double> angular_velocity_y_buffer;
    std::vector<double> angular_velocity_z_buffer;

    // // PitchrateOffsetStopEstimator variables
    PitchrateOffsetStopEstimator pitch_rate_offset_stop_estimator;
    double pitch_rate_offset;
    eagleye_msgs::msg::Status pitch_rate_offset_status;

    // PitchingEstimator variables
    PitchingEstimator pitching_estimator;
    double pitching;
    eagleye_msgs::msg::Status pitching_status;

    // AccelerationOffsetEstimator
    AccelerationOffsetEstimator acceleration_offset_estimator;    
    double filtered_acceleration;
    double acceleration_offset;
    eagleye_msgs::msg::Status acceleration_offset_status;

    //DopplerFusion
    double velocity;
    std::vector<double> time_buffer;
    std::vector<double> doppler_velocity_buffer;
    std::vector<double> corrected_acceleration_buffer;
    std::vector<double> rtklib_update_status_buffer;
    eagleye_msgs::msg::Status velocity_status;

    bool updateImu(const sensor_msgs::msg::Imu);
    bool updateRtklibNav(const rtklib_msgs::msg::RtklibNav);
    bool updateGGA(const nmea_msgs::msg::Gpgga);
    bool StopJudgment(const sensor_msgs::msg::Imu);
    bool DopplerFusion();
};

#endif /*NAVIGATION_H */
