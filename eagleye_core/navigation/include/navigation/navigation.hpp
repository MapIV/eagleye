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

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nmea_msgs/Gpgga.h"
#include "nmea_msgs/Gprmc.h"

#include "rtklib_msgs/RtklibNav.h"

#include "eagleye_msgs/Status.h"
#include "eagleye_msgs/StatusStamped.h"
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
  double imu_rate;
  double gnss_rate;
  double moving_judgement_threshold;
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
  double stop_judgement_threshold;
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
  double moving_judgement_threshold;
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
  double stop_judgement_threshold;
  double moving_judgement_threshold;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double curve_judgement_threshold;
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
  double stop_judgement_threshold;
  double slow_judgement_threshold;
  double update_distance;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double curve_judgement_threshold;
};

struct RtkHeadingStatus
{
  int tow_last;
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
  double stop_judgement_threshold;
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
  double tf_gnss_rotation_w;
  std::string tf_gnss_parent_frame;
  std::string tf_gnss_child_frame;

  double imu_rate;
  double gnss_rate;
  double moving_judgement_threshold;
  double estimated_interval;
  double update_distance;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double gnss_error_covariance;
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
  double stop_judgement_threshold;
  double sync_search_period;
  double proc_noise;
};

struct PositionInterpolateStatus
{
  int position_estimate_status_count;
  int number_buffer;
  bool position_estimate_start_status;
  bool is_estimate_start{false};
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

struct SlipangleParameter
{
  double manual_coefficient;
  double stop_judgement_threshold;
};

struct SlipCoefficientParameter
{
  double imu_rate;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double stop_judgement_threshold;
  double moving_judgement_threshold;
  double curve_judgement_threshold;
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
  double moving_judgement_threshold;
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
  double stop_judgement_threshold;
  double curve_judgement_threshold;
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
  double moving_judgement_threshold;
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

struct RtkDeadreckoningParameter
{
  double stop_judgement_threshold;
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
  double stop_judgement_threshold;
  double update_distance;
  double moving_average_time;
  double sync_judgement_threshold;
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
  double stop_judgement_threshold;
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

extern void velocity_scale_factor_estimate(const rtklib_msgs::RtklibNav, const geometry_msgs::TwistStamped, const VelocityScaleFactorParameter,
  VelocityScaleFactorStatus*, geometry_msgs::TwistStamped*, eagleye_msgs::VelocityScaleFactor*);
extern void velocity_scale_factor_estimate(const nmea_msgs::Gprmc, const geometry_msgs::TwistStamped, const VelocityScaleFactorParameter,
  VelocityScaleFactorStatus*, geometry_msgs::TwistStamped*, eagleye_msgs::VelocityScaleFactor*);
extern void distance_estimate(const geometry_msgs::TwistStamped, DistanceStatus*,eagleye_msgs::Distance*);
extern void yaw_rate_offset_stop_estimate(const geometry_msgs::TwistStamped, const sensor_msgs::Imu, const YawrateOffsetStopParameter,
  YawrateOffsetStopStatus*, eagleye_msgs::YawrateOffset*);
extern void yaw_rate_offset_estimate(const geometry_msgs::TwistStamped, const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,
  const sensor_msgs::Imu, const YawrateOffsetParameter, YawrateOffsetStatus*, eagleye_msgs::YawrateOffset*);
extern void heading_estimate(const rtklib_msgs::RtklibNav, const sensor_msgs::Imu, const geometry_msgs::TwistStamped,
  const eagleye_msgs::YawrateOffset, const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const HeadingParameter,
  HeadingStatus*,eagleye_msgs::Heading*);
extern void heading_estimate(const nmea_msgs::Gprmc, const sensor_msgs::Imu, const geometry_msgs::TwistStamped, const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const HeadingParameter, HeadingStatus*,eagleye_msgs::Heading*);
extern void heading_estimate(const eagleye_msgs::Heading, const sensor_msgs::Imu, const geometry_msgs::TwistStamped, const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const HeadingParameter, HeadingStatus*,eagleye_msgs::Heading*);
extern void position_estimate(const rtklib_msgs::RtklibNav, const geometry_msgs::TwistStamped, const eagleye_msgs::StatusStamped, const eagleye_msgs::Distance,
  const eagleye_msgs::Heading, const geometry_msgs::Vector3Stamped, const PositionParameter, PositionStatus*, eagleye_msgs::Position*);
extern void position_estimate(const nmea_msgs::Gpgga gga, const geometry_msgs::TwistStamped, const eagleye_msgs::StatusStamped, const eagleye_msgs::Distance, const eagleye_msgs::Heading,
  const geometry_msgs::Vector3Stamped, const PositionParameter, PositionStatus*, eagleye_msgs::Position*);
extern void slip_angle_estimate(const sensor_msgs::Imu,const geometry_msgs::TwistStamped,const eagleye_msgs::StatusStamped,const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::YawrateOffset,const SlipangleParameter,eagleye_msgs::SlipAngle*);
extern void slip_coefficient_estimate(const sensor_msgs::Imu,const rtklib_msgs::RtklibNav,const geometry_msgs::TwistStamped,
  const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const SlipCoefficientParameter,SlipCoefficientStatus*,double*);
extern void smoothing_estimate(const rtklib_msgs::RtklibNav,const geometry_msgs::TwistStamped,const SmoothingParameter,SmoothingStatus*,
  eagleye_msgs::Position*);
extern void trajectory_estimate(const sensor_msgs::Imu,const geometry_msgs::TwistStamped,const eagleye_msgs::StatusStamped,const eagleye_msgs::Heading,const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::YawrateOffset,const TrajectoryParameter,TrajectoryStatus*,geometry_msgs::Vector3Stamped*,eagleye_msgs::Position*,
  geometry_msgs::TwistStamped*, geometry_msgs::TwistWithCovarianceStamped*);
extern void heading_interpolate_estimate(const sensor_msgs::Imu,const geometry_msgs::TwistStamped,const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::YawrateOffset,const eagleye_msgs::Heading,const eagleye_msgs::SlipAngle,const HeadingInterpolateParameter,HeadingInterpolateStatus*,
  eagleye_msgs::Heading*);
extern void position_interpolate_estimate(const eagleye_msgs::Position,const geometry_msgs::Vector3Stamped,const eagleye_msgs::Position,
  const eagleye_msgs::Height,const eagleye_msgs::Heading,const PositionInterpolateParameter,PositionInterpolateStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);
extern void pitching_estimate(const sensor_msgs::Imu,const nmea_msgs::Gpgga,const geometry_msgs::TwistStamped,const eagleye_msgs::Distance,
  const HeightParameter,HeightStatus*,eagleye_msgs::Height*,eagleye_msgs::Pitching*,eagleye_msgs::AccXOffset*,eagleye_msgs::AccXScaleFactor*);
extern void trajectory3d_estimate(const sensor_msgs::Imu,const geometry_msgs::TwistStamped,const eagleye_msgs::StatusStamped,const eagleye_msgs::Heading,
  const eagleye_msgs::YawrateOffset,const eagleye_msgs::YawrateOffset,const eagleye_msgs::Pitching,const TrajectoryParameter,TrajectoryStatus*,
  geometry_msgs::Vector3Stamped*,eagleye_msgs::Position*,geometry_msgs::TwistStamped*, geometry_msgs::TwistWithCovarianceStamped*);
extern void rtk_dead_reckoning_estimate(const rtklib_msgs::RtklibNav,const geometry_msgs::Vector3Stamped,const nmea_msgs::Gpgga, const eagleye_msgs::Heading,
  const RtkDeadreckoningParameter,RtkDeadreckoningStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);
extern void rtk_dead_reckoning_estimate(const geometry_msgs::Vector3Stamped,const nmea_msgs::Gpgga, const eagleye_msgs::Heading,
  const RtkDeadreckoningParameter,RtkDeadreckoningStatus*,eagleye_msgs::Position*,sensor_msgs::NavSatFix*);
extern void rtk_heading_estimate(const nmea_msgs::Gpgga gga, const sensor_msgs::Imu, const geometry_msgs::TwistStamped, const eagleye_msgs::Distance,
  const eagleye_msgs::YawrateOffset, const eagleye_msgs::YawrateOffset,  const eagleye_msgs::SlipAngle, const eagleye_msgs::Heading, const RtkHeadingParameter,
  RtkHeadingStatus*,eagleye_msgs::Heading*);
extern void enable_additional_rolling_estimate(const geometry_msgs::TwistStamped, const eagleye_msgs::StatusStamped,const eagleye_msgs::YawrateOffset ,const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::Distance,const sensor_msgs::Imu,const geometry_msgs::PoseStamped,const eagleye_msgs::AngularVelocityOffset,
  const EnableAdditionalRollingParameter,EnableAdditionalRollingStatus*,eagleye_msgs::Rolling*,eagleye_msgs::AccYOffset*);
extern void rolling_estimate(const sensor_msgs::Imu,const geometry_msgs::TwistStamped,const eagleye_msgs::YawrateOffset,
  const eagleye_msgs::YawrateOffset,const RollingParameter,RollingStatus*,eagleye_msgs::Rolling*);


class VelocityEstimator
{
  public:
    VelocityEstimator();

    // Parameter setting
    void setParam(std::string yaml_file);

    // Main estimate function
    void VelocityEstimate(const sensor_msgs::Imu, const rtklib_msgs::RtklibNav, const nmea_msgs::Gpgga, geometry_msgs::TwistStamped*);

    eagleye_msgs::Status getStatus();

  private:

    class PitchrateOffsetStopEstimator
    {
      public:
        PitchrateOffsetStopEstimator();

        void setParam(std::string yaml_file);
        bool PitchrateOffsetStopEstimate(double pitchrate, double stop_status);

        double pitchrate_offset;
        eagleye_msgs::Status pitchrate_offset_status;

      private:
        struct Param
        {
          int imu_rate;
          int estimated_interval;
          int buffer_count_max;
        };
        PitchrateOffsetStopEstimator::Param param;

        int stop_count;
        std::vector<double> pitchrate_buffer;
    };

    class PitchingEstimator
    {
      public:
        PitchingEstimator();

        void setParam(std::string yaml_file);
        bool PitchingEstimate(double imu_time_last, double doppler_velocity, double rtkfix_velocity,
                              double pitchrate, double pitchrate_offset, double rtkfix_pitching,
                              bool navsat_update_status, bool stop_status);

        double pitching;
        eagleye_msgs::Status pitching_status;

      private:
        struct Param
        {
          double imu_rate;
          double gnss_rate;
          double estimated_interval;
          double buffer_max;
          double outlier_threshold;
          double estimated_velocity_threshold;
          double slow_judgement_threshold;
          double gnss_receiving_threshold;
          double estimated_gnss_coefficient;
          double outlier_ratio_threshold;
          double estimated_coefficient;
        };
        PitchingEstimator::Param param;

        std::vector<double> time_buffer;
        std::vector<double> corrected_pitchrate_buffer;
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
        eagleye_msgs::Status acceleration_offset_status;

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
      double stop_judgement_interval;
      double stop_judgement_velocity_threshold;
      double stop_judgement_buffer_maxnum;
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
    double pitchrate;
    double imu_time_last;

    // rtklib_nav variables
    double gnss_rate;
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

    // stop judgement variables
    bool stop_status;
    std::vector<double> angular_velocity_x_buffer;
    std::vector<double> angular_velocity_y_buffer;
    std::vector<double> angular_velocity_z_buffer;
 
    // // PitchrateOffsetStopEstimator variables
    PitchrateOffsetStopEstimator pitchrate_offset_stop_estimator;
    double pitchrate_offset;
    eagleye_msgs::Status pitchrate_offset_status;

    // PitchingEstimator variables
    PitchingEstimator pitching_estimator;
    double pitching;
    eagleye_msgs::Status pitching_status;

    // AccelerationOffsetEstimator
    AccelerationOffsetEstimator acceleration_offset_estimator;    
    double filtered_acceleration;
    double acceleration_offset;
    eagleye_msgs::Status acceleration_offset_status;

    //DopplerFusion
    double velocity;
    std::vector<double> time_buffer;
    std::vector<double> doppler_velocity_buffer;
    std::vector<double> corrected_acceleration_buffer;
    std::vector<double> rtklib_update_status_buffer;
    eagleye_msgs::Status velocity_status;

    bool updateImu(const sensor_msgs::Imu);
    bool updateRtklibNav(const rtklib_msgs::RtklibNav);
    bool updateGGA(const nmea_msgs::Gpgga);
    bool Stopjudgement(const sensor_msgs::Imu);
    bool DopplerFusion();
};


#endif /*NAVIGATION_H */
