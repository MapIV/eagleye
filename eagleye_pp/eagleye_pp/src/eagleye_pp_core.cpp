// Copyright (c) 2022, Map IV, Inc.
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

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"
#include "nmea2fix/nmea2fix.hpp"
#include "eagleye_pp.hpp"
#include <multi_rosbag_controller/multi_rosbag_controller.hpp>

/*****************
Constructor
******************/
eagleye_pp::eagleye_pp()
{
  data_length_ = 0;

  convert_height_num_ = 0;
  interval_plot_ = 5;
  interval_line_ = 1;

  output_kml_eagleye_forward_plot_ = false;
  output_kml_eagleye_backward_plot_ = false;
  output_kml_eagleye_pp_plot_ = false;
  output_kml_rtklib_plot_ = true;
  output_kml_gnss_plot_ = true;
  output_kml_eagleye_forward_line_ = true;
  output_kml_eagleye_backward_line_ = true;
  output_kml_eagleye_pp_line_ = true;

  heading_parameter_ = {};
  heading_interpolate_parameter_ = {};
  yawrate_offset_stop_parameter_ = {};
  yawrate_offset_1st_parameter_ = {};
  yawrate_offset_2nd_parameter_ = {};
  slip_angle_parameter_ = {};
  trajectory_parameter_ = {};
  velocity_scale_factor_parameter_ = {};
  position_parameter_ = {};
  position_interpolate_parameter_ = {};
  smoothing_parameter_ = {};
  height_parameter_ = {};
}


/***********************************************************************************
setOutputPath
Set member variable outputpath_
std::string arg_output_path: Output destination obtained from the launch file
************************************************************************************/
void eagleye_pp::setOutputPath(std::string arg_output_path)
{
 outputpath_ = arg_output_path;
}


/********************
setParam
*********************/
void eagleye_pp::setParam(YAML::Node arg_conf, std::string *arg_twist_topic, std::string *arg_imu_topic, std::string *arg_rtklib_nav_topic,  std::string *arg_nmea_sentence_topic)
{
  try
  {
    // Eagleye_pp params
    *arg_twist_topic = arg_conf["twist_topic"].as<std::string>();
    *arg_imu_topic = arg_conf["imu_topic"].as<std::string>();
    *arg_rtklib_nav_topic = arg_conf["rtklib_nav_topic"].as<std::string>();
    *arg_nmea_sentence_topic = arg_conf["nmea_sentence_topic"].as<std::string>();

    // output_log = arg_conf["output_log"].as<bool>();
    output_log_ = true;
    // timestamp_sort = arg_conf["timestamp_sort"].as<bool>();
    timestamp_sort_ = true;
    convert_height_num_ = arg_conf["convert_height_num"].as<int>();
    interval_line_ = arg_conf["interval_line"].as<double>();
    output_kml_eagleye_forward_plot_ = arg_conf["output_kml_eagleye_forward_plot"].as<bool>();
    output_kml_eagleye_backward_plot_ = arg_conf["output_kml_eagleye_backward_plot"].as<bool>();
    output_kml_eagleye_pp_plot_ = arg_conf["output_kml_eagleye_pp_plot"].as<bool>();
    output_kml_rtklib_plot_ = arg_conf["output_kml_rtklib_plot"].as<bool>();
    output_kml_gnss_plot_ = arg_conf["output_kml_gnss_plot"].as<bool>();
    output_kml_eagleye_forward_line_ = arg_conf["output_kml_eagleye_forward_line"].as<bool>();
    output_kml_eagleye_backward_line_ = arg_conf["output_kml_eagleye_backward_line"].as<bool>();
    output_kml_eagleye_pp_line_ = arg_conf["output_kml_eagleye_pp_line"].as<bool>();

    // eagleye_rt params

    use_gnss_mode_ = arg_conf["gnss"]["use_gnss_mode"].as<std::string>();
    use_nmea_downsample_ = arg_conf["gnss"]["use_nmea_downsample"].as<bool>();
    nmea_downsample_freq_ = arg_conf["gnss"]["nmea_downsample_freq"].as<double>();

    heading_interpolate_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    heading_interpolate_parameter_.stop_judgment_velocity_threshold = arg_conf["heading_interpolate"]["stop_judgment_velocity_threshold"].as<double>();
    heading_interpolate_parameter_.number_buffer_max = arg_conf["heading_interpolate"]["number_buffer_max"].as<int>();

    heading_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    heading_parameter_.estimated_number_min = arg_conf["heading"]["estimated_number_min"].as<double>();
    heading_parameter_.estimated_number_max = arg_conf["heading"]["estimated_number_max"].as<double>();
    heading_parameter_.estimated_gnss_coefficient = arg_conf["heading"]["estimated_gnss_coefficient"].as<double>();
    heading_parameter_.estimated_heading_coefficient = arg_conf["heading"]["estimated_heading_coefficient"].as<double>();
    heading_parameter_.outlier_threshold = arg_conf["heading"]["outlier_threshold"].as<double>();
    heading_parameter_.estimated_velocity_threshold = arg_conf["heading"]["estimated_velocity_threshold"].as<double>();
    heading_parameter_.stop_judgment_velocity_threshold = arg_conf["heading"]["stop_judgment_velocity_threshold"].as<double>();
    heading_parameter_.estimated_yawrate_threshold = arg_conf["heading"]["estimated_yawrate_threshold"].as<double>();

    position_interpolate_parameter_.number_buffer_max = arg_conf["position_interpolate"]["number_buffer_max"].as<int>();

    position_parameter_.estimated_distance = arg_conf["position"]["estimated_distance"].as<double>();
    position_parameter_.separation_distance = arg_conf["position"]["separation_distance"].as<double>();
    position_parameter_.estimated_velocity_threshold = arg_conf["position"]["estimated_velocity_threshold"].as<double>();
    position_parameter_.outlier_threshold = arg_conf["position"]["outlier_threshold"].as<double>();
    position_parameter_.estimated_enu_vel_coefficient = arg_conf["position"]["estimated_enu_vel_coefficient"].as<double>();
    position_parameter_.estimated_position_coefficient = arg_conf["position"]["estimated_position_coefficient"].as<double>();;
    position_parameter_.ecef_base_pos_x = arg_conf["position"]["ecef_base_pos_x"].as<double>();
    position_parameter_.ecef_base_pos_y = arg_conf["position"]["ecef_base_pos_y"].as<double>();
    position_parameter_.ecef_base_pos_z = arg_conf["position"]["ecef_base_pos_z"].as<double>();

    slip_angle_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    slip_angle_parameter_.manual_coefficient = arg_conf["slip_angle"]["manual_coefficient"].as<double>();
    slip_angle_parameter_.stop_judgment_velocity_threshold = arg_conf["slip_angle"]["stop_judgment_velocity_threshold"].as<double>();

    smoothing_parameter_.ecef_base_pos_x = arg_conf["position"]["ecef_base_pos_x"].as<double>();
    smoothing_parameter_.ecef_base_pos_y = arg_conf["position"]["ecef_base_pos_y"].as<double>();
    smoothing_parameter_.ecef_base_pos_z = arg_conf["position"]["ecef_base_pos_z"].as<double>();
    smoothing_parameter_.estimated_number_max = arg_conf["smoothing"]["estimated_number_max"].as<int>();
    smoothing_parameter_.estimated_velocity_threshold = arg_conf["smoothing"]["estimated_velocity_threshold"].as<double>();
    smoothing_parameter_.estimated_threshold = arg_conf["smoothing"]["estimated_threshold"].as<double>();

    trajectory_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    trajectory_parameter_.stop_judgment_velocity_threshold = arg_conf["trajectory"]["stop_judgment_velocity_threshold"].as<double>();

    velocity_scale_factor_parameter_.estimated_number_min = arg_conf["velocity_scale_factor"]["estimated_number_min"].as<int>();
    velocity_scale_factor_parameter_.estimated_number_max = arg_conf["velocity_scale_factor"]["estimated_number_max"].as<int>();
    velocity_scale_factor_parameter_.estimated_velocity_threshold = arg_conf["velocity_scale_factor"]["estimated_velocity_threshold"].as<double>();
    velocity_scale_factor_parameter_.estimated_coefficient = arg_conf["velocity_scale_factor"]["estimated_coefficient"].as<double>();

    yawrate_offset_1st_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    yawrate_offset_1st_parameter_.estimated_number_min = arg_conf["yawrate_offset"]["estimated_number_min"].as<int>();
    yawrate_offset_1st_parameter_.estimated_coefficient = arg_conf["yawrate_offset"]["estimated_coefficient"].as<double>();
    yawrate_offset_1st_parameter_.estimated_velocity_threshold = arg_conf["yawrate_offset"]["estimated_velocity_threshold"].as<double>();
    yawrate_offset_1st_parameter_.estimated_number_max = arg_conf["yawrate_offset"]["1st"]["estimated_number_max"].as<int>();
    yawrate_offset_1st_parameter_.outlier_threshold = arg_conf["yawrate_offset"]["outlier_threshold"].as<double>();

    yawrate_offset_2nd_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    yawrate_offset_2nd_parameter_.estimated_number_min = arg_conf["yawrate_offset"]["estimated_number_min"].as<int>();
    yawrate_offset_2nd_parameter_.estimated_coefficient = arg_conf["yawrate_offset"]["estimated_coefficient"].as<double>();
    yawrate_offset_2nd_parameter_.estimated_velocity_threshold = arg_conf["yawrate_offset"]["estimated_velocity_threshold"].as<double>();
    yawrate_offset_2nd_parameter_.estimated_number_max = arg_conf["yawrate_offset"]["2nd"]["estimated_number_max"].as<int>();
    yawrate_offset_2nd_parameter_.outlier_threshold = arg_conf["yawrate_offset"]["outlier_threshold"].as<double>();

    yawrate_offset_stop_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    yawrate_offset_stop_parameter_.stop_judgment_velocity_threshold = arg_conf["yawrate_offset_stop"]["stop_judgment_velocity_threshold"].as<double>();
    yawrate_offset_stop_parameter_.estimated_number = arg_conf["yawrate_offset_stop"]["estimated_number"].as<int>();
    yawrate_offset_stop_parameter_.outlier_threshold = arg_conf["yawrate_offset_stop"]["outlier_threshold"].as<double>();

    height_parameter_.estimated_distance = arg_conf["height"]["estimated_distance"].as<double>();
    height_parameter_.estimated_distance_max = arg_conf["height"]["estimated_distance_max"].as<int>();
    height_parameter_.separation_distance = arg_conf["height"]["separation_distance"].as<double>();
    height_parameter_.estimated_velocity_threshold = arg_conf["height"]["estimated_velocity_threshold"].as<double>();
    height_parameter_.estimated_velocity_coefficient = arg_conf["height"]["estimated_velocity_coefficient"].as<double>();
    height_parameter_.estimated_height_coefficient = arg_conf["height"]["estimated_height_coefficient"].as<double>();
    height_parameter_.outlier_threshold = arg_conf["height"]["outlier_threshold"].as<double>();
    height_parameter_.average_num = arg_conf["height"]["average_num"].as<int>();

    rolling_parameter_.reverse_imu = arg_conf["reverse_imu"].as<bool>();
    rolling_parameter_.stop_judgment_velocity_threshold = arg_conf["rolling"]["stop_judgment_velocity_threshold"].as<double>();
    rolling_parameter_.filter_process_noise = arg_conf["rolling"]["filter_process_noise"].as<double>();
    rolling_parameter_.filter_observation_noise = arg_conf["rolling"]["filter_observation_noise"].as<double>();
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[31;1mYAML Error: " << e.what() << "\033[m" << std::endl;
    exit(3);
  }
}


/******************************************
setDataLength
Set member variable data_length_
*******************************************/
void eagleye_pp::setDataLength(void)
{
 data_length_ = std::distance(imu_.begin(), imu_.end());
}


std::size_t eagleye_pp::getDataLength(void)
{
 return data_length_;
}


std::string eagleye_pp::getUseGNSSMode(void)
{
 return use_gnss_mode_;
}


std::vector<rtklib_msgs::RtklibNav> eagleye_pp::getRtklibNavVector(void)
{
 return rtklib_nav_;
}

bool eagleye_pp::getUseBackward(void)
{
 return (output_kml_eagleye_backward_line_ || output_kml_eagleye_backward_plot_ || getUseCombination());
}

bool eagleye_pp::getUseCombination(void)
{
 return (output_kml_eagleye_pp_line_ || output_kml_eagleye_pp_plot_);
}

/****************************************************************
syncTimestamp
Function to synchronize time
bool arg_nmea_data_flag : Determining if there is nmea
rosbag::View& arg_in_view : rosbag data
******************************************************************/
void eagleye_pp::syncTimestamp(bool arg_nmea_data_flag, rosbag::View& arg_in_view)
{
  std::vector<long int> rosbag_stamp;

  std::vector<rtklib_msgs::RtklibNav> tmp_rtklib_nav;
  std::vector<sensor_msgs::NavSatFix> tmp_fix;
  std::vector<geometry_msgs::TwistStamped> tmp_velocity;
  std::vector<nmea_msgs::Sentence> tmp_nmea_sentence;
  std::vector<nmea_msgs::Gpgga> tmp_gga;
  std::vector<nmea_msgs::Gprmc> tmp_rmc;

  rtklib_msgs::RtklibNav rtklib_nav_msg_last;
  sensor_msgs::NavSatFix fix_msg_last;
  geometry_msgs::TwistStamped velocity_msg_last;
  nmea_msgs::Sentence nmea_sentence_msg_last;
  nmea_msgs::Gpgga gga_msg_last;
  nmea_msgs::Gprmc rmc_msg_last;

  double fix_time_last;
  double rmc_time_last;

  std::cout << "data loading... " << std::endl;
  BOOST_FOREACH(rosbag::MessageInstance m, arg_in_view)
  {
    sensor_msgs::Imu::ConstPtr m_imu_msg = m.instantiate<sensor_msgs::Imu>();
    rtklib_msgs::RtklibNav::ConstPtr m_rtklib_nav_msg = m.instantiate<rtklib_msgs::RtklibNav>();
    sensor_msgs::NavSatFix::ConstPtr m_fix_msg = m.instantiate<sensor_msgs::NavSatFix>();
    geometry_msgs::TwistStamped::ConstPtr m_velocity_msg = m.instantiate<geometry_msgs::TwistStamped>();
    nmea_msgs::Sentence::ConstPtr m_nmea_sentence_msg = m.instantiate<nmea_msgs::Sentence>();

    if (timestamp_sort_)
    {
      if (m_imu_msg != NULL)
      {
        sensor_msgs::Imu imu_msg = *m_imu_msg;
        imu_.push_back(imu_msg);
        rosbag_stamp.push_back(m.getTime().toNSec());
      }
      if (m_rtklib_nav_msg != NULL)
      {
        rtklib_msgs::RtklibNav rtklib_nav_msg = *m_rtklib_nav_msg;
        tmp_rtklib_nav.push_back(rtklib_nav_msg);
      }
      if (arg_nmea_data_flag && m_nmea_sentence_msg != NULL)
      {
        nmea_msgs::Sentence nmea_sentence_msg = *m_nmea_sentence_msg;
        nmea_msgs::Gpgga gga_msg;
        nmea_msgs::Gprmc rmc_msg;
        sensor_msgs::NavSatFix fix_msg;
        tmp_nmea_sentence.push_back(nmea_sentence_msg);
        nmea2fix_converter(nmea_sentence_msg, &fix_msg, &gga_msg, &rmc_msg);

        if(use_nmea_downsample_)
        {
          if (gga_msg.header.stamp.toSec() != 0 && std::abs(gga_msg.header.stamp.toSec()-fix_time_last) > (1/nmea_downsample_freq_))
          {
            tmp_gga.push_back(gga_msg);
            tmp_fix.push_back(fix_msg);
            fix_time_last = gga_msg.header.stamp.toSec();
          }
          if (rmc_msg.header.stamp.toSec() != 0 && std::abs(rmc_msg.header.stamp.toSec()-rmc_time_last) > (1/nmea_downsample_freq_))
          {
            tmp_rmc.push_back(rmc_msg);
            rmc_time_last = rmc_msg.header.stamp.toSec();
          }
        }
        else
        {
          if (gga_msg.header.stamp.toSec() != 0)
          {
            tmp_gga.push_back(gga_msg);
            tmp_fix.push_back(fix_msg);
          }
          if (rmc_msg.header.stamp.toSec() != 0)
          {
            tmp_rmc.push_back(rmc_msg);
          }
        }
        
      }
      if (m_velocity_msg != NULL)
      {
        geometry_msgs::TwistStamped velocity_msg = *m_velocity_msg;
        tmp_velocity.push_back(velocity_msg);
      }
    }
    else
    {
      if (m_rtklib_nav_msg != NULL)
      {
        rtklib_msgs::RtklibNav rtklib_nav_msg = *m_rtklib_nav_msg;
        rtklib_nav_msg_last = rtklib_nav_msg;
      }
      if (arg_nmea_data_flag)
      {
        if (m_nmea_sentence_msg != NULL)
        {
          nmea_msgs::Sentence nmea_sentence_msg = *m_nmea_sentence_msg;
          nmea_msgs::Gpgga gga_msg;
          nmea_msgs::Gprmc rmc_msg;
          sensor_msgs::NavSatFix fix_msg;
          nmea_sentence_msg_last = nmea_sentence_msg;
          nmea2fix_converter(nmea_sentence_msg, &fix_msg, &gga_msg, &rmc_msg);

          if(use_nmea_downsample_)
          {
            if (gga_msg.header.stamp.toSec() != 0 && std::abs(gga_msg.header.stamp.toSec()-fix_time_last) > (1/nmea_downsample_freq_))
            {
              gga_msg_last = gga_msg;
              fix_msg_last = fix_msg;
              fix_time_last = gga_msg.header.stamp.toSec();
            }
            if (rmc_msg.header.stamp.toSec() != 0 && std::abs(rmc_msg.header.stamp.toSec()-rmc_time_last) > (1/nmea_downsample_freq_))
            {
              rmc_msg_last = rmc_msg;
              rmc_time_last = rmc_msg.header.stamp.toSec();
            }
          }
          else
          {
            if (gga_msg.header.stamp.toSec() != 0)
            {
              gga_msg_last = gga_msg;
              fix_msg_last = fix_msg;
            }
            if (rmc_msg.header.stamp.toSec() != 0)
            {
              rmc_msg_last = rmc_msg;
            }
          }
        }
      }
      if (m_velocity_msg != NULL)
      {
        geometry_msgs::TwistStamped velocity_msg = *m_velocity_msg;
        velocity_msg_last = velocity_msg;
      }
      if (m_imu_msg != NULL)
      {
        sensor_msgs::Imu imu_msg = *m_imu_msg;
        imu_.push_back(imu_msg);
        rtklib_nav_.push_back(rtklib_nav_msg_last);
        gga_.push_back(gga_msg_last);
        rmc_.push_back(rmc_msg_last);
        nmea_sentence_.push_back(nmea_sentence_msg_last);
        velocity_.push_back(velocity_msg_last);
        rosbag_stamp.push_back(m.getTime().toNSec());
      }
    }
  }

  std::size_t tmp_rtklib_nav_length;
  std::size_t tmp_gga_length;
  std::size_t tmp_rmc_length;
  std::size_t tmp_velocity_length;

  setDataLength();
  tmp_rtklib_nav_length = std::distance(tmp_rtklib_nav.begin(), tmp_rtklib_nav.end());
  tmp_gga_length = std::distance(tmp_gga.begin(), tmp_gga.end());
  tmp_rmc_length = std::distance(tmp_rmc.begin(), tmp_rmc.end());
  tmp_velocity_length = std::distance(tmp_velocity.begin(), tmp_velocity.end());

  std::vector<double> rtklib_nav_stamp;
  for (int i=0; i<tmp_rtklib_nav_length; i++)
  {
    rtklib_nav_stamp.push_back(tmp_rtklib_nav[i].header.stamp.toSec());
  }

  std::vector<double> gga_stamp;
  for (int i=0; i<tmp_gga_length; i++)
  {
    gga_stamp.push_back(tmp_gga[i].header.stamp.toSec());
  }

  std::vector<double> rmc_stamp;
  for (int i=0; i<tmp_rmc_length; i++)
  {
    rmc_stamp.push_back(tmp_rmc[i].header.stamp.toSec());
  }

  std::vector<double> velocity_stamp;
  for (int i=0; i<tmp_velocity_length; i++)
  {
    velocity_stamp.push_back(tmp_velocity[i].header.stamp.toSec());
  }

  std::vector<double>::iterator rtklib_nav_stamp_position;
  std::vector<double>::iterator gga_stamp_position;
  std::vector<double>::iterator rmc_stamp_position;
  std::vector<double>::iterator velocity_stamp_position;
  int rtklib_nav_stamp_index;
  int gga_stamp_index;
  int rmc_stamp_index;
  int velocity_stamp_index;

  if (timestamp_sort_)
  {
    rtklib_nav_.resize(data_length_);
    gga_.resize(data_length_);
    velocity_.resize(data_length_);
    if(arg_nmea_data_flag)
    {
      gga_.resize(data_length_);
      rmc_.resize(data_length_);
    }

    for (int i=0; i<data_length_; i++)
    {
      rtklib_nav_stamp_position = std::lower_bound(rtklib_nav_stamp.begin(), rtklib_nav_stamp.end(), imu_[i].header.stamp.toSec());
      gga_stamp_position = std::lower_bound(gga_stamp.begin(), gga_stamp.end(), imu_[i].header.stamp.toSec());
      rmc_stamp_position = std::lower_bound(rmc_stamp.begin(), rmc_stamp.end(), imu_[i].header.stamp.toSec());
      velocity_stamp_position = std::lower_bound(velocity_stamp.begin(), velocity_stamp.end(), imu_[i].header.stamp.toSec());
      
      rtklib_nav_stamp_index = std::distance(rtklib_nav_stamp.begin(), rtklib_nav_stamp_position);
      gga_stamp_index = std::distance(gga_stamp.begin(), gga_stamp_position);
      rmc_stamp_index = std::distance(rmc_stamp.begin(), rmc_stamp_position);
      velocity_stamp_index = std::distance(velocity_stamp.begin(), velocity_stamp_position);
      if (rtklib_nav_stamp_index != 0)
      {
        rtklib_nav_[i]=tmp_rtklib_nav[rtklib_nav_stamp_index-1];
      }

      if (gga_stamp_index != 0)
      {
        gga_[i]=tmp_gga[gga_stamp_index-1];
        if(arg_nmea_data_flag) gga_[i]=tmp_gga[gga_stamp_index-1];
      }

      if (rmc_stamp_index != 0)
      {
        if(arg_nmea_data_flag) rmc_[i]=tmp_rmc[rmc_stamp_index-1];
      }

      if (velocity_stamp_index != 0)
      {
        velocity_[i]=tmp_velocity[velocity_stamp_index-1];
      }
    }
  }
}


/**************************************************
estimatingEagleye
Function to calculate the estimate
bool forwardFlg　: true by forward, false by back
***************************************************/
void eagleye_pp::estimatingEagleye(bool arg_forward_flag)
{
  struct DistanceStatus distance_status{};
  struct HeadingStatus heading_1st_status{};
  struct HeadingStatus heading_2nd_status{};
  struct HeadingStatus heading_3rd_status{};
  struct HeadingInterpolateStatus heading_interpolate_1st_status{};
  struct HeadingInterpolateStatus heading_interpolate_2nd_status{};
  struct HeadingInterpolateStatus heading_interpolate_3rd_status{};
  struct YawrateOffsetStopStatus yawrate_offset_stop_status{};
  struct YawrateOffsetStatus yawrate_offset_1st_status{};
  struct YawrateOffsetStatus yawrate_offset_2nd_status{};
  struct HeightStatus height_status{},b_height_status{};
  struct RollingStatus rolling_status{},b_rolling_status{};
  struct TrajectoryStatus trajectory_status{};
  struct VelocityScaleFactorStatus velocity_scale_factor_status{};
  struct PositionStatus position_status{};
  struct PositionInterpolateStatus position_interpolate_status{};
  struct SmoothingStatus smoothing_status{};

  eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
  eagleye_msgs::Distance _distance;
  eagleye_msgs::Heading _heading_1st;
  eagleye_msgs::Heading _heading_interpolate_1st;
  eagleye_msgs::Heading _heading_2nd;
  eagleye_msgs::Heading _heading_interpolate_2nd;
  eagleye_msgs::Heading _heading_3rd;
  eagleye_msgs::Heading _heading_interpolate_3rd;
  eagleye_msgs::YawrateOffset _yawrate_offset_stop;
  eagleye_msgs::YawrateOffset _yawrate_offset_1st;
  eagleye_msgs::YawrateOffset _yawrate_offset_2nd;
  eagleye_msgs::SlipAngle _slip_angle;
  eagleye_msgs::Height _height;
  eagleye_msgs::Pitching _pitching;
  eagleye_msgs::AccXOffset _acc_x_offset;
  eagleye_msgs::AccXScaleFactor _acc_x_scale_factor;
  eagleye_msgs::Rolling _rolling;
  eagleye_msgs::Position _enu_relative_pos;
  geometry_msgs::Vector3Stamped _enu_vel;
  eagleye_msgs::Position _enu_absolute_pos;
  eagleye_msgs::Position _enu_absolute_pos_interpolate;
  eagleye_msgs::Position _gnss_smooth_pos_enu;
  sensor_msgs::NavSatFix _eagleye_fix;
  geometry_msgs::TwistStamped _eagleye_twist;


  int last_prog = -1, current_prog;
  int i = 0;
  for (int k = 0; k < data_length_; k++)
  {
    current_prog = static_cast<int>((k+1) / static_cast<double>(data_length_) * 100);
    if(arg_forward_flag)
    {
      i = k;
      if (current_prog > last_prog)
      {
        std::cout << "forward estimation process..." << current_prog << " %" << "\r" << std::flush;
      }
    }
    else
    {
      i = data_length_-1 - k;
      if (current_prog > last_prog)
      {
        std::cout << "backward estimation process..." << current_prog << " %" << "\r" << std::flush;
      }
    }

    _velocity_scale_factor.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      velocity_scale_factor_estimate(rtklib_nav_[i], velocity_[i], velocity_scale_factor_parameter_, &velocity_scale_factor_status, &_velocity_scale_factor);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      velocity_scale_factor_estimate(rmc_[i], velocity_[i], velocity_scale_factor_parameter_, &velocity_scale_factor_status, &_velocity_scale_factor);

    _slip_angle.header = imu_[i].header;
    slip_angle_estimate(imu_[i], _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_2nd, slip_angle_parameter_, &_slip_angle);

    _gnss_smooth_pos_enu.header = imu_[i].header;
    smoothing_estimate(rtklib_nav_[i], _velocity_scale_factor, smoothing_parameter_, &smoothing_status, &_gnss_smooth_pos_enu);

    _yawrate_offset_stop.header = imu_[i].header;
    yawrate_offset_stop_estimate(velocity_[i], imu_[i], yawrate_offset_stop_parameter_, &yawrate_offset_stop_status, &_yawrate_offset_stop);

    _heading_1st.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      heading_estimate(rtklib_nav_[i], imu_[i],_velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_stop, _slip_angle, _heading_interpolate_1st, heading_parameter_, &heading_1st_status, &_heading_1st);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      heading_estimate(rmc_[i], imu_[i],_velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_stop, _slip_angle, _heading_interpolate_1st, heading_parameter_, &heading_1st_status, &_heading_1st);

    _heading_interpolate_1st.header = imu_[i].header;
    heading_interpolate_estimate(imu_[i], _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_stop, _heading_1st, _slip_angle, heading_interpolate_parameter_, &heading_interpolate_1st_status, &_heading_interpolate_1st);

    _yawrate_offset_1st.header = imu_[i].header;
    yawrate_offset_estimate(_velocity_scale_factor, _yawrate_offset_stop, _heading_interpolate_1st, imu_[i], yawrate_offset_1st_parameter_, &yawrate_offset_1st_status, &_yawrate_offset_1st);

    _heading_2nd.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      heading_estimate(rtklib_nav_[i], imu_[i], _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_1st, _slip_angle, _heading_interpolate_2nd, heading_parameter_, &heading_2nd_status, &_heading_2nd);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      heading_estimate(rmc_[i], imu_[i], _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_1st, _slip_angle, _heading_interpolate_2nd, heading_parameter_, &heading_2nd_status, &_heading_2nd);

    _heading_interpolate_2nd.header = imu_[i].header;
    heading_interpolate_estimate(imu_[i], _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_1st, _heading_2nd, _slip_angle, heading_interpolate_parameter_, &heading_interpolate_2nd_status, &_heading_interpolate_2nd);

    _yawrate_offset_2nd.header = imu_[i].header;
    yawrate_offset_estimate(_velocity_scale_factor, _yawrate_offset_stop, _heading_interpolate_2nd, imu_[i], yawrate_offset_2nd_parameter_, &yawrate_offset_2nd_status, &_yawrate_offset_2nd);

    _heading_3rd.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      heading_estimate(rtklib_nav_[i], imu_[i], _velocity_scale_factor, _yawrate_offset_2nd, _yawrate_offset_stop, _slip_angle, _heading_interpolate_3rd, heading_parameter_, &heading_3rd_status, &_heading_3rd);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      heading_estimate(rmc_[i], imu_[i], _velocity_scale_factor, _yawrate_offset_2nd, _yawrate_offset_stop, _slip_angle, _heading_interpolate_3rd, heading_parameter_, &heading_3rd_status, &_heading_3rd);

    _heading_interpolate_3rd.header = imu_[i].header;
    heading_interpolate_estimate(imu_[i], _velocity_scale_factor, _yawrate_offset_2nd, _yawrate_offset_stop, _heading_3rd, _slip_angle, heading_interpolate_parameter_, &heading_interpolate_3rd_status, &_heading_interpolate_3rd);

    _distance.header = imu_[i].header;
    if(_distance.header.stamp.toSec() != 0)
    {
      distance_estimate(_velocity_scale_factor, &distance_status, &_distance);
    }

    _height.header = imu_[i].header;
    _pitching.header = imu_[i].header;
    _acc_x_offset.header = imu_[i].header;
    _acc_x_scale_factor.header = imu_[i].header;
    pitching_estimate(imu_[i], gga_[i], _velocity_scale_factor, _distance, height_parameter_, &height_status, &_height, &_pitching, &_acc_x_offset, &_acc_x_scale_factor);

    rolling_estimate(imu_[i], _velocity_scale_factor, _yawrate_offset_stop, _yawrate_offset_2nd, rolling_parameter_, &rolling_status, &_rolling);

    _enu_vel.header = imu_[i].header;
    _enu_relative_pos.header = imu_[i].header;
    _eagleye_twist.header = imu_[i].header;
    trajectory_estimate(imu_[i], _velocity_scale_factor, _heading_interpolate_3rd, _yawrate_offset_stop, _yawrate_offset_2nd, trajectory_parameter_, &trajectory_status, &_enu_vel, &_enu_relative_pos, &_eagleye_twist);

    _enu_absolute_pos.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      position_estimate(rtklib_nav_[i], _velocity_scale_factor, _distance, _heading_interpolate_3rd, _enu_vel, position_parameter_, &position_status, &_enu_absolute_pos);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      position_estimate(gga_[i], _velocity_scale_factor, _distance, _heading_interpolate_3rd, _enu_vel, position_parameter_, &position_status, &_enu_absolute_pos);

    _enu_absolute_pos_interpolate.header = imu_[i].header;
    _eagleye_fix.header = imu_[i].header;
    position_interpolate_estimate(_enu_absolute_pos, _enu_vel, _gnss_smooth_pos_enu, _height, position_interpolate_parameter_, &position_interpolate_status, &_enu_absolute_pos_interpolate, &_eagleye_fix);

    if(arg_forward_flag)
    {
    	eagleye_state_forward_.velocity_scale_factor.push_back(_velocity_scale_factor);
    	eagleye_state_forward_.distance.push_back(_distance);
    	eagleye_state_forward_.heading_1st.push_back(_heading_1st);
    	eagleye_state_forward_.heading_interpolate_1st.push_back(_heading_interpolate_1st);
    	eagleye_state_forward_.heading_2nd.push_back(_heading_2nd);
    	eagleye_state_forward_.heading_interpolate_2nd.push_back(_heading_interpolate_2nd);
    	eagleye_state_forward_.heading_3rd.push_back(_heading_3rd);
    	eagleye_state_forward_.heading_interpolate_3rd.push_back(_heading_interpolate_3rd);
    	eagleye_state_forward_.yawrate_offset_stop.push_back(_yawrate_offset_stop);
    	eagleye_state_forward_.yawrate_offset_1st.push_back(_yawrate_offset_1st);
    	eagleye_state_forward_.yawrate_offset_2nd.push_back(_yawrate_offset_2nd);
    	eagleye_state_forward_.slip_angle.push_back(_slip_angle);
    	eagleye_state_forward_.height.push_back(_height);
    	eagleye_state_forward_.pitching.push_back(_pitching);
    	eagleye_state_forward_.rolling.push_back(_rolling);
    	eagleye_state_forward_.acc_x_offset.push_back(_acc_x_offset);
    	eagleye_state_forward_.acc_x_scale_factor.push_back(_acc_x_scale_factor);
    	eagleye_state_forward_.enu_relative_pos.push_back(_enu_relative_pos);
    	eagleye_state_forward_.enu_vel.push_back(_enu_vel);
    	eagleye_state_forward_.enu_absolute_pos.push_back(_enu_absolute_pos);
    	eagleye_state_forward_.enu_absolute_pos_interpolate.push_back(_enu_absolute_pos_interpolate);
    	eagleye_state_forward_.gnss_smooth_pos_enu.push_back(_gnss_smooth_pos_enu);
    	eagleye_state_forward_.eagleye_fix.push_back(_eagleye_fix);
    	eagleye_state_forward_.eagleye_twist.push_back(_eagleye_twist);
    	eagleye_state_forward_.flag_reliability_buffer.push_back(height_status.flag_reliability);
    }
    else
    {
	    eagleye_state_backward_.velocity_scale_factor.push_back(_velocity_scale_factor);
    	eagleye_state_backward_.distance.push_back(_distance);
    	eagleye_state_backward_.heading_1st.push_back(_heading_1st);
    	eagleye_state_backward_.heading_interpolate_1st.push_back(_heading_interpolate_1st);
    	eagleye_state_backward_.heading_2nd.push_back(_heading_2nd);
    	eagleye_state_backward_.heading_interpolate_2nd.push_back(_heading_interpolate_2nd);
    	eagleye_state_backward_.heading_3rd.push_back(_heading_3rd);
    	eagleye_state_backward_.heading_interpolate_3rd.push_back(_heading_interpolate_3rd);
    	eagleye_state_backward_.yawrate_offset_stop.push_back(_yawrate_offset_stop);
    	eagleye_state_backward_.yawrate_offset_1st.push_back(_yawrate_offset_1st);
    	eagleye_state_backward_.yawrate_offset_2nd.push_back(_yawrate_offset_2nd);
    	eagleye_state_backward_.slip_angle.push_back(_slip_angle);
    	eagleye_state_backward_.height.push_back(_height);
    	eagleye_state_backward_.pitching.push_back(_pitching);
    	eagleye_state_backward_.rolling.push_back(_rolling);
    	eagleye_state_backward_.acc_x_offset.push_back(_acc_x_offset);
    	eagleye_state_backward_.acc_x_scale_factor.push_back(_acc_x_scale_factor);
    	eagleye_state_backward_.enu_relative_pos.push_back(_enu_relative_pos);
    	eagleye_state_backward_.enu_vel.push_back(_enu_vel);
    	eagleye_state_backward_.enu_absolute_pos.push_back(_enu_absolute_pos);
    	eagleye_state_backward_.enu_absolute_pos_interpolate.push_back(_enu_absolute_pos_interpolate);
    	eagleye_state_backward_.gnss_smooth_pos_enu.push_back(_gnss_smooth_pos_enu);
    	eagleye_state_backward_.eagleye_fix.push_back(_eagleye_fix);
    	eagleye_state_backward_.eagleye_twist.push_back(_eagleye_twist);
    	eagleye_state_backward_.flag_reliability_buffer.push_back(height_status.flag_reliability);
    }

    _slip_angle.status.estimate_status = false;
    _heading_1st.status.estimate_status = false;
    _yawrate_offset_1st.status.estimate_status = false;
    _heading_2nd.status.estimate_status = false;
    _yawrate_offset_2nd.status.estimate_status = false;
    _heading_3rd.status.estimate_status = false;
    _enu_absolute_pos.status.estimate_status = false;
    _height.status.estimate_status = false;
    _pitching.status.estimate_status = false;
    _acc_x_offset.status.estimate_status = false;
    _acc_x_scale_factor.status.estimate_status = false;
  }// end for  

  if(!arg_forward_flag)
  { 
    std::reverse(eagleye_state_backward_.velocity_scale_factor.begin(), eagleye_state_backward_.velocity_scale_factor.end());
    std::reverse(eagleye_state_backward_.distance.begin(), eagleye_state_backward_.distance.end());
    std::reverse(eagleye_state_backward_.heading_1st.begin(), eagleye_state_backward_.heading_1st.end());
    std::reverse(eagleye_state_backward_.heading_interpolate_1st.end(), eagleye_state_backward_.heading_interpolate_1st.end());
    std::reverse(eagleye_state_backward_.heading_2nd.begin(), eagleye_state_backward_.heading_2nd.end());
    std::reverse(eagleye_state_backward_.heading_interpolate_2nd.begin(), eagleye_state_backward_.heading_interpolate_2nd.end());
    std::reverse(eagleye_state_backward_.heading_3rd.begin(), eagleye_state_backward_.heading_3rd.end());
    std::reverse(eagleye_state_backward_.heading_interpolate_3rd.begin(), eagleye_state_backward_.heading_interpolate_3rd.end());
    std::reverse(eagleye_state_backward_.yawrate_offset_stop.begin(), eagleye_state_backward_.yawrate_offset_stop.end());
    std::reverse(eagleye_state_backward_.yawrate_offset_1st.begin(), eagleye_state_backward_.yawrate_offset_1st.end());
    std::reverse(eagleye_state_backward_.yawrate_offset_2nd.begin(), eagleye_state_backward_.yawrate_offset_2nd.end());
    std::reverse(eagleye_state_backward_.slip_angle.begin(), eagleye_state_backward_.slip_angle.end());
    std::reverse(eagleye_state_backward_.height.begin(), eagleye_state_backward_.height.end());
    std::reverse(eagleye_state_backward_.pitching.begin(), eagleye_state_backward_.pitching.end());
    std::reverse(eagleye_state_backward_.acc_x_offset.begin(), eagleye_state_backward_.acc_x_offset.end());
    std::reverse(eagleye_state_backward_.acc_x_scale_factor.begin(), eagleye_state_backward_.acc_x_scale_factor.end());
    std::reverse(eagleye_state_backward_.enu_relative_pos.begin(), eagleye_state_backward_.enu_relative_pos.end());
    std::reverse(eagleye_state_backward_.enu_vel.begin(), eagleye_state_backward_.enu_vel.end());
    std::reverse(eagleye_state_backward_.enu_absolute_pos.begin(), eagleye_state_backward_.enu_absolute_pos.end());
    std::reverse(eagleye_state_backward_.enu_absolute_pos_interpolate.begin(), eagleye_state_backward_.enu_absolute_pos_interpolate.end());
    std::reverse(eagleye_state_backward_.gnss_smooth_pos_enu.begin(), eagleye_state_backward_.gnss_smooth_pos_enu.end());
    std::reverse(eagleye_state_backward_.eagleye_fix.begin(), eagleye_state_backward_.eagleye_fix.end());
    std::reverse(eagleye_state_backward_.eagleye_twist.begin(), eagleye_state_backward_.eagleye_twist.end());
    std::reverse(eagleye_state_backward_.flag_reliability_buffer.begin(), eagleye_state_backward_.flag_reliability_buffer.end());
  }
}

/***********************
setGPSTime
A function that sets the time used for azimuth calculation
arg_GPSTime　: output,time used for azimuth calculation
***********************/
void eagleye_pp::setGPSTime(double arg_GPSTime[]){
double *GNSSTime = (double*)malloc(sizeof(double) * data_length_);
  std::vector<int>  index_gnsstime;
  for(int i =0; i < data_length_; i++){
    GNSSTime[i] = (double)rtklib_nav_[i].tow / 1000;
  }
  for(int i =1; i < data_length_; i++){
    if(GNSSTime[i] != GNSSTime[i-1]){
	index_gnsstime.push_back(i);
	arg_GPSTime[i] = GNSSTime[i];
    }
  }
  for(int i =1; i < index_gnsstime.size(); i++){
    double diff_time = GNSSTime[index_gnsstime[i]] - GNSSTime[index_gnsstime[i-1]];
    int diff_cnt = index_gnsstime[i] - index_gnsstime[i-1];
    double time = diff_time/diff_cnt;
    for(int j =0; j < diff_cnt; j++){
      arg_GPSTime[index_gnsstime[i-1]+j+1] = arg_GPSTime[index_gnsstime[i-1]+j] + time;
    }
  }
//Setting a value outside the range of index_gnsstime
  for(int i =0; i < index_gnsstime[0]; i++){
    arg_GPSTime[i] = arg_GPSTime[index_gnsstime[0]];
  }
  for(int i = index_gnsstime[index_gnsstime.size()-1]; i < data_length_; i++){
    arg_GPSTime[i] = arg_GPSTime[index_gnsstime[index_gnsstime.size()-1]];
  }
}

/***********************************************
calcMissPositiveFIX
Function to remove missFix 
double arg_TH_POSMAX : Threshold
double arg_GPSTime   : Corrected time based on the time obtained from GNSS
***********************************************/
void eagleye_pp::calcMissPositiveFIX(double arg_TH_POSMAX, double arg_GPSTime[]){

  double TH_VEL_EST = 10/3.6;
  int ESTDIST = 50;
  double TH_CALC_MINNUM = 0.01;
  std::size_t datanum = data_length_;
  std::vector<bool> flag_Elim(data_length_, 0);

  flag_GNSS_.resize(datanum);
  for(int i = 0; i < datanum; i++){ //高さによる判定が不明なためfix判定のみで実装
    if(rtklib_nav_[i].status.status.status == 0){
	  flag_GNSS_[i] = 1;
    }
  }
  std::vector<int> index_Raw;
  int a = 0;
  index_Raw.resize(datanum);
  for(int i = 0; i < datanum; i++){
    if(eagleye_state_forward_.distance[i].status.estimate_status == true){
      index_Raw[a] = i;
      a++;
    }
  }
  std::vector<double> _distance(datanum, 0.0);

  for(int i = 1; i < datanum; i++){
    _distance[i] = _distance[i-1] + eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x * (arg_GPSTime[i] - arg_GPSTime[i-1]);
  }
  for(int i = 0; i < datanum; i++){
    int index_Dist = -1;
    for(int k = 0; k < datanum; k++){
      if(_distance[k] > _distance[i] - ESTDIST){
        index_Dist = k;
        break;
      }
    }

    if (_distance[i] > ESTDIST && flag_GNSS_[i] == 1 && eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x > TH_VEL_EST && index_Dist > index_Raw[0]){  
      int ESTNUM = i - index_Dist + 1;
      int i_start = i-ESTNUM + 1;
      int local_length = ESTNUM;//(i - i_start)+1;
      double pUsrPos_enu[local_length][3] = {0};
      double pUsrVel_enu[local_length][3] = {0};
      double pGPSTime[local_length] = {0};
      for(int k = 0; k < local_length; k++){
	  pUsrPos_enu[k][0] = eagleye_state_forward_.enu_absolute_pos_interpolate[i_start + k].enu_pos.x;
	  pUsrPos_enu[k][1] = eagleye_state_forward_.enu_absolute_pos_interpolate[i_start + k].enu_pos.y;
	  pUsrPos_enu[k][2] = eagleye_state_forward_.enu_absolute_pos_interpolate[i_start + k].enu_pos.z;
	  pUsrVel_enu[k][0] = eagleye_state_forward_.enu_vel[i_start + k].vector.x;
	  pUsrVel_enu[k][1] = eagleye_state_forward_.enu_vel[i_start + k].vector.y;
	  pUsrVel_enu[k][2] = eagleye_state_forward_.enu_vel[i_start + k].vector.z;
	  pGPSTime[k] = arg_GPSTime[i_start + k];
      }
      std::vector<int> pindex_GNSS;
      for(int j = 0; j < local_length; j++){
	if(flag_GNSS_[i_start + j] == 1 ){
	    pindex_GNSS.push_back(j);
	}
      }
      std::vector<int> index_Elim;
      for(int j = 0; j< local_length; j++){
	index_Elim.push_back(i_start + j);
      }        
      std::vector<int> pindex_vel;
      for(int j = 0; j < local_length; j++){ 
	if(eagleye_state_forward_.velocity_scale_factor[i_start + j].correction_velocity.linear.x > TH_VEL_EST ){
	    pindex_vel.push_back(j);
	}
      }

      size_t pindex_vel_length = pindex_vel.size();
      std::vector<int> index;
      std::set_intersection(pindex_GNSS.begin(), pindex_GNSS.end(), pindex_vel.begin(), pindex_vel.end(), std::inserter(index, index.end()));
      if (index.size() > pindex_vel_length*TH_CALC_MINNUM){
	double tTrajectory[ESTNUM][3]={0};
        for(int j = 1; j < ESTNUM; j++){
          tTrajectory[j][0] = tTrajectory[j-1][0] + pUsrVel_enu[j][0]*(pGPSTime[j] - pGPSTime[j-1]);
	  tTrajectory[j][1] = tTrajectory[j-1][1] + pUsrVel_enu[j][1]*(pGPSTime[j] - pGPSTime[j-1]);
	  tTrajectory[j][2] = tTrajectory[j-1][2] + pUsrVel_enu[j][2]*(pGPSTime[j] - pGPSTime[j-1]);
	}

        while (1){
          double basepos[ESTNUM][2] = {0};
	  for(int k = 0; k < ESTNUM; k++){		
	    basepos[k][0] = pUsrPos_enu[index[index.size()-1]][0] + tTrajectory[k][0]-tTrajectory[index[index.size()-1]][0] ;
	    basepos[k][1] = pUsrPos_enu[index[index.size()-1]][1] + tTrajectory[k][1]-tTrajectory[index[index.size()-1]][1] ;
	  }
          double pdiff2[index.size()][2] = {0};
	  for(int k = 0; k < index.size(); k++){
	    pdiff2[k][0] = basepos[index[k]][0] - pUsrPos_enu[index[k]][0];
	    pdiff2[k][1] = basepos[index[k]][1] - pUsrPos_enu[index[k]][1];
	  }
    double tUsrPos_enu[2]={0};
	  double avg_pdiff2[2] = {0};
	  for(int k = 0; k < index.size(); k++){
	    avg_pdiff2[0] += pdiff2[k][0];
	    avg_pdiff2[1] += pdiff2[k][1];
	  }
	  avg_pdiff2[0] /= index.size();
	  avg_pdiff2[1] /= index.size();
	  tUsrPos_enu[0] = pUsrPos_enu[index[index.size()-1]][0] - avg_pdiff2[0];
	  tUsrPos_enu[1] = pUsrPos_enu[index[index.size()-1]][1] - avg_pdiff2[1];
	        
    double basepos2[ESTNUM][2]={0};
	  double pdiff[index.size()][2]={0};
	  for(int k = 0; k < ESTNUM; k++){
	    basepos2[k][0] = tUsrPos_enu[0] + tTrajectory[k][0]-tTrajectory[index[index.size()-1]][0];                      
	    basepos2[k][1] = tUsrPos_enu[1] + tTrajectory[k][1]-tTrajectory[index[index.size()-1]][1];      
          }
	  for(int k = 0; k < index.size(); k++){
	    pdiff[k][0] = basepos2[index[k]][0] - pUsrPos_enu[index[k]][0];
	    pdiff[k][1] = basepos2[index[k]][1] - pUsrPos_enu[index[k]][1];
	  }

	  double Y[2] = {abs(pdiff[0][0]), abs(pdiff[0][1])};
	  int I[2] = {0, 0};
	  double Y_large = 0;
	  int I_large = 0;
	  for(int k = 1; k < index.size(); k++){ //max(abs(pdiff))     
	    if(Y[0] < abs(pdiff[k][0])){
	      Y[0] = abs(pdiff[k][0]);
	      I[0] = k;
	    }
	    if(Y[1] < abs(pdiff[k][1])){
	      Y[1] = abs(pdiff[k][1]);
	      I[1] = k;
	    }
	  }
	  if (Y[0] < Y[1]){
	    Y_large = Y[1];
	    I_large = I[1];
	  }else if(Y[0] > Y[1]){
	    Y_large = Y[0];
	    I_large = I[0];
          }    
          if (Y_large > arg_TH_POSMAX){
            flag_Elim[index_Elim[index[I_large]]] = 1;
            index.erase(index.begin() + I_large);
	    if(index.size() < 1){break;}
          }else{
            break;
          }       
        } // while(1)  
      } // if (index.size() > pindex_vel_length*TH_CALC_MINNUM)
    } // if (_distance[i] > ESTDIST && flag_GNSS_[i] == 1 && velocity_scale_factor_[i].correction_velocity.linear.x > TH_VEL_EST && index_Dist > index_Raw[0])
  } // for(int i = 0; i < datanum; i++){
  
  int kk = 0;
  while(kk < data_length_){
    if(flag_Elim[kk] == 1){ 
	flag_GNSS_[kk] = 0;
    }
    kk++;
  }

}

/*********************************************************************
calcPickDR
function to get the DRs and DRe flags
double arg_GPSTime		: Corrected time based on the time obtained from GNSS
bool arg_flag_SMRaw		: output, flag of smoothingRaw
std::vector<int> &arg_index_DRs : output, Start of DR index 
std::vector<int> &arg_index_DRe : output, End of DR index
**********************************************************************/
void eagleye_pp::calcPickDR(double arg_GPSTime[], bool *arg_flag_SMRaw, std::vector<int> &arg_index_DRs, std::vector<int> &arg_index_DRe){

  const int ESTNUM_MIN = 10;
  const int ESTNUM_MAX = 25;
  const double ESTNUM_GNSF = 0.04;
  const double DRdis = 200;

  int estnum = 0;
  std::size_t datanum = data_length_;

  std::vector<bool> flag_DRs(datanum, 0);
  std::vector<bool> flag_DRe(datanum, 0);


  for(int i = 0; i < datanum; i++){ 
     if (i > ESTNUM_MIN && flag_GNSS_[i] == 1){
        if (i > ESTNUM_MIN){
            estnum = ESTNUM_MIN;
        }else{
            estnum = i;
        }
	std::vector<int> index;
	for(int k = (i-estnum+1); k <= i; k++){
	  if(flag_GNSS_[k] == 1){
	    index.push_back(k);
	  }
	}
    	std::size_t index_length = std::distance(index.begin(), index.end());
    	if (index_length > estnum * ESTNUM_GNSF){
           arg_flag_SMRaw[i] = 1;
       	}
     }
  }

  // Pick up Long DR
  std::vector<double> _distance(datanum, 0.0);

  for(int i = 1; i < datanum; i++){
    _distance[i] = _distance[i-1] + eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x * (arg_GPSTime[i] - arg_GPSTime[i-1]);
  }
  for(int i = 0; i < datanum; i++){
    if (arg_flag_SMRaw[i] == 1){
      for (int ii = 1; ii < datanum; ii++){
        if (i + ii <= datanum){
          if (arg_flag_SMRaw[i + ii] == 1){
            if (_distance[i] - _distance[i + ii] < -DRdis){
              flag_DRs[i] = 1;
              flag_DRe[i + ii] = 1;
            }
            break;
          }
        }
      } // for (int ii = 1; ii < datanum; ii++)
    }
  }

  for(int i = 0; i <= datanum; i++){
    if(flag_DRs[i] == 1){
      arg_index_DRs.push_back(i);
    } 	
  }
  for(int i = 0; i <= datanum; i++){
    if(flag_DRe[i] == 1){
      arg_index_DRe.push_back(i);
    } 	
  }
}


/**********************************************************************
calcInitialHeading
Function to calculate the initial azimuth
double arg_GPSTime	       : Corrected time based on the time obtained from GNSS
bool arg_flag_SMRaw            : flag of 2DSmoothingrRaw
std::vector<int> arg_index_DRs : Start of DR index
std::vector<int> arg_index_DRe : End of DR index
***********************************************************************/
void eagleye_pp::calcInitialHeading(double arg_GPSTime[], bool arg_flag_SMRaw[], std::vector<int> arg_index_DRs, std::vector<int> arg_index_DRe){

  const double deltaHead_start = -5.0; //deltaHead= -5: 0.05 :5;
  const double deltaHead_range = 0.05; //
  const double deltaHead_end   = 5.0;  //
  const bool Tramodelswitch = 0;
  const double TH_Yaw = 1 * M_PI/180;

  int deltaHead_length = static_cast<int>((deltaHead_end - deltaHead_start) / deltaHead_range)+1;
  std::vector<double> deltaHead(deltaHead_length, 0);
  int aa = 1;
  deltaHead[0] = deltaHead_start;
  while(aa < deltaHead_length){
    deltaHead[aa] = deltaHead[aa-1] + deltaHead_range;
    aa++;
  }
  std::size_t datanum = data_length_;
  std::vector<double> UsrPos_TaGRTK_enu(datanum*3, 0);
  for(int i = 0; i < datanum; i++){
    UsrPos_TaGRTK_enu[i + datanum * 0] = eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.x;
    UsrPos_TaGRTK_enu[i + datanum * 1] = eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.y;
    UsrPos_TaGRTK_enu[i + datanum * 2] = eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.z;
  }
  std::vector<double> Heading(datanum, 0.0);
  std::vector<bool> index_Heading;
  for(int i = 0; i < datanum; i++){
    Heading[i] = eagleye_state_forward_.heading_interpolate_3rd[i].heading_angle;
    if(eagleye_state_forward_.heading_interpolate_3rd[i].status.enabled_status == true){
    	index_Heading.push_back(i);
    }
  }
  std::vector<double> Yawrate_Est(datanum, 0.0);
  std::vector<double> slip(datanum, 0.0);
  for(int i = 0; i < datanum; i++){
    Yawrate_Est[i] = eagleye_state_forward_.eagleye_twist[i].twist.angular.z;
    slip[i] = eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x * Yawrate_Est[i] * slip_angle_parameter_.manual_coefficient;
  }
  std::size_t DRerr_length = arg_index_DRs.size() * 6 + 6;
  // double **DRerr, *DRerr_row;
  // DRerr = (double**)malloc(sizeof(double *) * deltaHead_length);
  // DRerr_row = (double*)malloc(sizeof(double) * deltaHead_length * DRerr_length);
  std::vector<double> DRerr(arg_index_DRs.size() * 6 + 6);
  // std::vector<double> DRerr_row(deltaHead_length * DRerr_length);
  // for (int i=0;i<deltaHead_length;i++) {
  //   DRerr[i] = DRerr_row + i * DRerr_length;
  // }
  double InitHeadingOffset[arg_index_DRs.size()][2] = {0.0};
  std::vector<double> Heading2(datanum, 0);
  for(int i = 0; i < datanum; i++){
    Heading2[i] = Heading[i];
  }
  std::vector<bool> flag_DRs(datanum, false);
  for(int i = 0; i < arg_index_DRs.size(); i++){
    flag_DRs[arg_index_DRs[i]] = 1;
  }
  std::vector<double> Heading_IMU_slip(datanum, false);
  for(int I = 0; I < deltaHead_length; I++){
    for(int p = 0; p < arg_index_DRs.size(); p++){
      Heading[arg_index_DRs[p]] = Heading2[arg_index_DRs[p]] + (deltaHead[I]*M_PI/180);
    }
    std::vector<double> Heading_IMU(datanum, 0);
    for(int i = 0; i < datanum; i++){
      if (i == index_Heading[0] || flag_DRs[i] == 1){
        Heading_IMU_slip[i]  = Heading[i];
      }
      else if (i > 0){
        Heading_IMU_slip[i] = Heading_IMU_slip[i-1] + (Yawrate_Est[i]) * ( arg_GPSTime[i] - arg_GPSTime[i-1] ); 
      }
    }
    for(int i = 0; i < datanum; i++){
      if (arg_flag_SMRaw[i] == 0){
        Heading_IMU_slip[i] = Heading_IMU_slip[i] - slip[i];
      }
    }

    std::vector<double> pUsrPos_FixSlip(datanum * 2, 0);
    int a = 0;
    int switch_tmp = 0;
    for(int i = 0; i < datanum; i++){
      if( arg_index_DRs.size() > 1){
        if(flag_DRs[i] == 1 && i > arg_index_DRs[1]){
          a = a + 1;   
        }      
      }
      double correction_velocity_x = eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x;
      if (i == 0){
        pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i + datanum * 0];
        pUsrPos_FixSlip[i + datanum * 1] = UsrPos_TaGRTK_enu[i + datanum * 1];
      }else if( flag_DRs[i] == 1 && i != arg_index_DRe[a]){ 
        switch_tmp = 0;    
        pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i + datanum * 0];
        pUsrPos_FixSlip[i + datanum * 1] = UsrPos_TaGRTK_enu[i + datanum * 1];
      }else if(flag_DRs[i] == 1 && i == arg_index_DRe[a] ){
        switch_tmp = 1;
        if (Tramodelswitch == 0){  
          pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
          pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
        }else if( Tramodelswitch == 1){    
          if(abs(Yawrate_Est[i]) > TH_Yaw){
             pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
             pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
          }else{
            pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
          }
        } // else if( Tramodelswitch == 1)
      }else if( i > 0){
        if(Tramodelswitch == 0){  
          if(switch_tmp == 0){
            pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
          }else if(switch_tmp == 1){
            if(flag_DRs[i-1] == 1){
              pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              pUsrPos_FixSlip[i + datanum * 1]= UsrPos_TaGRTK_enu[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            }else{
              pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[I] - arg_GPSTime[i-1]);
            }
          } // else if(switch_tmp == 1)
        }else if(Tramodelswitch == 1){     
          if(switch_tmp == 0){
            if(abs(Yawrate_Est[i]) > TH_Yaw){
              pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
              pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
            }else{
              pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            }
          }else if(switch_tmp == 1){
            if (flag_DRs[i-1] == 1){
              if (abs(Yawrate_Est[i]) > TH_Yaw){
                pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i-1 + datanum * 0] + ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
                pUsrPos_FixSlip[i + datanum * 1]= UsrPos_TaGRTK_enu[i-1 + datanum * 1] + ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
              }else{
                pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
                pUsrPos_FixSlip[i + datanum * 1]= UsrPos_TaGRTK_enu[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              }
	    }else{ 
              if(abs(Yawrate_Est[i]) > TH_Yaw){
                pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
                pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
              }else{
                pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] + sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
                pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] + cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              }
            }
          } // else if(switch_tmp == 1)
        } // else if(Tramodelswitch == 1)
      } // else if( i > 0)
    } // for(int i = 0; i < datanum; i++)
    std::vector<double> pPosdiffslip_e(arg_index_DRe.size(), 0);
    std::vector<double> pPosdiffslip_n(arg_index_DRe.size(), 0);
    std::vector<double> pPosdiffslip_2D(arg_index_DRe.size(), 0);

    for(int i = 0; i < arg_index_DRe.size(); i++){
      pPosdiffslip_e[i] = pUsrPos_FixSlip[arg_index_DRe[i] + datanum * 0] -UsrPos_TaGRTK_enu[arg_index_DRe[i] + datanum * 0];
      pPosdiffslip_n[i] = pUsrPos_FixSlip[arg_index_DRe[i] + datanum * 1] -UsrPos_TaGRTK_enu[arg_index_DRe[i] + datanum * 1];
      pPosdiffslip_2D[i] = sqrt(std::pow(pPosdiffslip_e[i],2) + std::pow(pPosdiffslip_n[i],2));
      
    }
    for (int i = 0; i < arg_index_DRe.size(); i++){
      DRerr[I * i*6] = i;
      DRerr[I * i*6+1] = I;
      DRerr[I * i*6+2] = deltaHead[I];
      DRerr[I * i*6+3] = pPosdiffslip_e[i];
      DRerr[I * i*6+4] = pPosdiffslip_n[i];
      DRerr[I * i*6+5] = pPosdiffslip_2D[i];
    }
  }// for(int I = 0; I < deltaHead_length; I++)
  for(int i = 0; i < arg_index_DRe.size(); i++){
    int min = 0;
    for(int k = 1; k < deltaHead_length; k++){
      if(DRerr[min * (i*6)+5] > DRerr[k * (i*6)+5]){
      min = k;
      }
    }
    InitHeadingOffset[i][0] = i;
    InitHeadingOffset[i][1] = DRerr[min * (i*6)+2];
  }
  std::vector<double> Heading3(datanum, 0);

  for(int i = 0; i < datanum; i++){
    Heading3[i] = Heading2[i];
  }
  for(int i = 0; i < arg_index_DRs.size(); i++){
    Heading2[arg_index_DRs[i]] = Heading3[arg_index_DRs[i]] + InitHeadingOffset[i][1]*M_PI/180;
  }
  for(int i = 0; i < datanum; i++){
    eagleye_state_forward_.heading_interpolate_3rd[i].heading_angle = Heading2[i]; //output
  }

}

void eagleye_pp::smoothingDeadReckoning()
{
  std::size_t data_length = getDataLength();
  // Calculate initial azimuth
  double GPSTime[data_length] = {0};

  double *GNSSTime = (double*)malloc(sizeof(double) * data_length);
  std::vector<int>  index_gnsstime;
  std::vector<rtklib_msgs::RtklibNav> rtklib_nav_vector = getRtklibNavVector();
  for(int i =0; i < data_length; i++){
    GNSSTime[i] = (double)rtklib_nav_vector[i].tow / 1000;
  }
  for(int i =1; i < data_length; i++){
    if(GNSSTime[i] != GNSSTime[i-1]){
      index_gnsstime.push_back(i);
      GPSTime[i] = GNSSTime[i];
    }
  }
  for(int i =1; i < index_gnsstime.size(); i++){
    double diff_time = GNSSTime[index_gnsstime[i]] - GNSSTime[index_gnsstime[i-1]];
    int diff_cnt = index_gnsstime[i] - index_gnsstime[i-1];
    double time = diff_time/diff_cnt;
    for(int j =0; j < diff_cnt; j++){
      GPSTime[index_gnsstime[i-1]+j+1] = GPSTime[index_gnsstime[i-1]+j] + time;
    }
  }
  for(int i =0; i < index_gnsstime[0]; i++){
    GPSTime[i] = GPSTime[index_gnsstime[0]];
  }
  for(int i = index_gnsstime[index_gnsstime.size()-1]; i < data_length; i++){
    GPSTime[i] = GPSTime[index_gnsstime[index_gnsstime.size()-1]];
  }
  free(GNSSTime);

  std::vector<int> index_DRs;
  std::vector<int> index_DRe;
  std::cout << std::endl << "Start MissPositiveFIX" <<  std::endl;
  bool flag_SMRaw_2D[data_length] = {0};
  double TH_POSMAX;
  // if(loop_count == 1)
  // {
  //   TH_POSMAX = 1.5;
  // }else
  // {
    TH_POSMAX = 0.3;
  // }
  calcMissPositiveFIX(TH_POSMAX, GPSTime);
  std::cout << "Start PickDR" <<  std::endl;
  calcPickDR(GPSTime, flag_SMRaw_2D, index_DRs, index_DRe);
  std::cout << "Start initial azimuth calculation";
  calcInitialHeading(GPSTime, flag_SMRaw_2D, index_DRs, index_DRe);
}



/*******************************
smoothingTrajectory
Function to smooth the orbit
*******************************/
void eagleye_pp::smoothingTrajectory(void)
{
  int i,j;
  int interval_count;
  int index = 0;
  double tmp_llh[3],tmp_ecef[3],tmp_enu[3],tmp_ecef_base[3], tmp_enu_f[3],tmp_enu_r[3];
  double enu_smoothing_trajectory[3], llh_smoothing_trajectory[3];
  double tmp_vel_begin,tmp_vle_end,all_vel_begin,all_vel_end;
  double tmp_sum_vel, all_sum_vel;
  double llh_gga[3];
  double diff_pos_east,diff_pos_north,diff_pos_2d;
  double time_last = 0;
  double outlier_num_1 = 0.3,outlier_num_2 = 1.5;
  bool start_estimate_status = false;
  std::vector<int> index_gga;
  std::vector<double> tmp_vel,all_vel,diff_time;
  std::vector<double> trajectory_ef,trajectory_er,trajectory_nf,trajectory_nr;
  std::vector<double> ratio;
  std::size_t gga_length;
  std::size_t enu_relative_pos_length;
  std::size_t enu_absolute_pos_length;
  std::size_t velocity_scale_factor_length;
  std::size_t flag_reliability_buffer_length;
  std::size_t index_gga_length;
  std::size_t enu_smoothing_trajectory_length;

  struct TrajectoryStatus trajectory_status{};
  for(int i = 0; i < data_length_; i++){ // Added to use the corrected initial azimuth
    trajectory_estimate(imu_[i], eagleye_state_forward_.velocity_scale_factor[i], eagleye_state_forward_.heading_interpolate_3rd[i], 
      eagleye_state_forward_.yawrate_offset_stop[i], eagleye_state_forward_.yawrate_offset_2nd[i],
      trajectory_parameter_, &trajectory_status, &eagleye_state_forward_.enu_vel[i], &eagleye_state_forward_.enu_relative_pos[i],
      &eagleye_state_forward_.eagleye_twist[i]);
  }

  gga_length = std::distance(gga_.begin(), gga_.end());
  enu_relative_pos_length = std::distance(eagleye_state_forward_.enu_relative_pos.begin(), eagleye_state_forward_.enu_relative_pos.end());
  enu_absolute_pos_length = std::distance(eagleye_state_forward_.enu_absolute_pos.begin(), eagleye_state_forward_.enu_absolute_pos.end());
  velocity_scale_factor_length = std::distance(eagleye_state_forward_.velocity_scale_factor.begin(), eagleye_state_forward_.velocity_scale_factor.end());
  flag_reliability_buffer_length = std::distance(eagleye_state_forward_.flag_reliability_buffer.begin(), eagleye_state_forward_.flag_reliability_buffer.end());

  if(gga_length > 0 && gga_length == enu_relative_pos_length && gga_length == enu_absolute_pos_length && gga_length == velocity_scale_factor_length)
  {
    if (eagleye_state_forward_.enu_absolute_pos[enu_absolute_pos_length-1].status.enabled_status == true)
    {
      tmp_ecef_base[0] = eagleye_state_forward_.enu_absolute_pos[enu_absolute_pos_length-1].ecef_base_pos.x;
      tmp_ecef_base[1] = eagleye_state_forward_.enu_absolute_pos[enu_absolute_pos_length-1].ecef_base_pos.y;
      tmp_ecef_base[2] = eagleye_state_forward_.enu_absolute_pos[enu_absolute_pos_length-1].ecef_base_pos.z;

      for(i = 0; i < gga_length; i++)
      {
        if(eagleye_state_forward_.flag_reliability_buffer[i] == true && eagleye_state_forward_.enu_relative_pos[i].status.enabled_status == true &&
          time_last != gga_[i].header.stamp.toSec() && gga_[i].gps_qual ==4 )
        // if(enu_relative_pos[i].status.enabled_status == true && time_last != gga[i].header.stamp.toSec() && gga[i].status.status !=-1 )
        {
          index_gga.push_back(i);
          time_last = gga_[i].header.stamp.toSec();
          start_estimate_status = true;
        }
      }

      index_gga_length = std::distance(index_gga.begin(), index_gga.end());

      if(index_gga_length > 0)
      {
        tmp_llh[0] = gga_[index_gga[0]].lat *M_PI/180;
        tmp_llh[1] = gga_[index_gga[0]].lon*M_PI/180;
        tmp_llh[2] = gga_[index_gga[0]].alt + gga_[index_gga[0]].undulation;

        llh2xyz(tmp_llh, tmp_ecef);
        xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu);

        for(i = 0; i < index_gga[0]; i++)
        {
          if(gga_[i].header.stamp.toSec() != 0)
          {
            if(time_last != gga_[i].header.stamp.toSec() &&gga_[i].gps_qual ==4)
            {
              index = i;
              time_last = gga_[i].header.stamp.toSec();
              tmp_llh[0] = gga_[i].lat *M_PI/180;
              tmp_llh[1] = gga_[i].lon*M_PI/180;
              tmp_llh[2] = gga_[i].alt + gga_[i].undulation;

              llh2xyz(tmp_llh, tmp_ecef);
              xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu);
            }

            if(index >0)
            {
              smoothing_trajectory_status_.push_back(2);
              enu_smoothing_trajectory_east_.push_back(eagleye_state_backward_.enu_relative_pos[i].enu_pos.x + tmp_enu[0] -
                eagleye_state_backward_.enu_relative_pos[index].enu_pos.x);
              enu_smoothing_trajectory_north_.push_back(eagleye_state_backward_.enu_relative_pos[i].enu_pos.y + tmp_enu[1] -
                eagleye_state_backward_.enu_relative_pos[index].enu_pos.y);
              enu_smoothing_trajectory_height_.push_back(tmp_enu[2]);
            }
            else
            {
              smoothing_trajectory_status_.push_back(-1);
              enu_smoothing_trajectory_east_.push_back(0);
              enu_smoothing_trajectory_north_.push_back(0);
              enu_smoothing_trajectory_height_.push_back(0);
            }
          }
          else
          {
            smoothing_trajectory_status_.push_back(2);
            enu_smoothing_trajectory_east_.push_back(eagleye_state_backward_.enu_relative_pos[i].enu_pos.x + tmp_enu[0] -
              eagleye_state_backward_.enu_relative_pos[index_gga[0]].enu_pos.x);
            enu_smoothing_trajectory_north_.push_back(eagleye_state_backward_.enu_relative_pos[i].enu_pos.y + tmp_enu[1] -
              eagleye_state_backward_.enu_relative_pos[index_gga[0]].enu_pos.y);
            enu_smoothing_trajectory_height_.push_back(tmp_enu[2]);
          }
        }

        for(i = 1; i < index_gga_length; i++)
        {
          tmp_llh[0] = gga_[index_gga[i-1]].lat *M_PI/180;
          tmp_llh[1] = gga_[index_gga[i-1]].lon*M_PI/180;
          tmp_llh[2] = gga_[index_gga[i-1]].alt + gga_[index_gga[i-1]].undulation;

          llh2xyz(tmp_llh, tmp_ecef);
          xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu_f);

          tmp_llh[0] = gga_[index_gga[i]].lat *M_PI/180;
          tmp_llh[1] = gga_[index_gga[i]].lon*M_PI/180;
          tmp_llh[2] = gga_[index_gga[i]].alt + gga_[index_gga[i]].undulation;

          llh2xyz(tmp_llh, tmp_ecef);
          xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu_r);
          enu2llh(tmp_enu_r, tmp_ecef_base, llh_gga);

          interval_count =  index_gga[i] - index_gga[i-1] ;

          for(j = 0; j < interval_count; j++)
          {
            diff_time.push_back(eagleye_state_forward_.velocity_scale_factor[index_gga[i-1]+1 + j].header.stamp.toSec() -
              eagleye_state_forward_.velocity_scale_factor[index_gga[i-1]+1 + j-1].header.stamp.toSec());
            tmp_vel.push_back(eagleye_state_forward_.velocity_scale_factor[index_gga[i-1]+1 + j].correction_velocity.linear.x);
            all_vel.push_back(eagleye_state_forward_.velocity_scale_factor[index_gga[i-1]+1 + j].correction_velocity.linear.x);
          }

          for(j = 0; j < interval_count; j++)
          {
            trajectory_ef.push_back(eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.x + tmp_enu_f[0] -
              eagleye_state_forward_.enu_relative_pos[index_gga[i-1]].enu_pos.x);
            trajectory_nf.push_back(eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.y + tmp_enu_f[1] -
              eagleye_state_forward_.enu_relative_pos[index_gga[i-1]].enu_pos.y);
            trajectory_er.push_back(eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.x + tmp_enu_r[0] -
              eagleye_state_forward_.enu_relative_pos[index_gga[i-1] + interval_count].enu_pos.x);
            trajectory_nr.push_back(eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.y + tmp_enu_r[1] -
              eagleye_state_forward_.enu_relative_pos[index_gga[i-1] + interval_count].enu_pos.y);

            if (j == 1)
            {
              diff_pos_east = (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.x + tmp_enu_f[0] -
                eagleye_state_forward_.enu_relative_pos[index_gga[i-1]].enu_pos.x) -
                (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.x + tmp_enu_r[0] - eagleye_state_forward_.enu_relative_pos[index_gga[i-1] + interval_count].enu_pos.x);
              diff_pos_north = (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.y + tmp_enu_f[1] -
                eagleye_state_forward_.enu_relative_pos[index_gga[i-1]].enu_pos.y) -
                (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.y + tmp_enu_r[1] - eagleye_state_forward_.enu_relative_pos[index_gga[i-1] + interval_count].enu_pos.y);
              diff_pos_2d = sqrt( pow((diff_pos_east), 2.0) + pow((diff_pos_north), 2.0));
            }

            tmp_vel_begin = eagleye_state_forward_.velocity_scale_factor[index_gga[i-1]+1].correction_velocity.linear.x;
            tmp_vle_end = eagleye_state_forward_.velocity_scale_factor[index_gga[i-1] + j-1].correction_velocity.linear.x;
            all_vel_begin  = eagleye_state_forward_.velocity_scale_factor[index_gga[i-1]+1].correction_velocity.linear.x;
            all_vel_end = eagleye_state_forward_.velocity_scale_factor[index_gga[i]].correction_velocity.linear.x;

            tmp_sum_vel = std::accumulate(tmp_vel.begin(), tmp_vel.begin() + j, 0.0);
            all_sum_vel = std::accumulate(all_vel.begin(), all_vel.end(), 0.0);

            if(j > 0 && all_sum_vel != 0)
            {
              ratio.push_back(tmp_sum_vel/all_sum_vel);
            }
            else
            {
              ratio.push_back(0);
            }

            if(ratio[j] * diff_pos_2d < outlier_num_1 || (1-ratio[j]) * diff_pos_2d < outlier_num_1)
            {
              smoothing_trajectory_status_.push_back(0);
            }
            else if(ratio[j] * diff_pos_2d < outlier_num_2 || (1-ratio[j]) * diff_pos_2d < outlier_num_2)
            {
              smoothing_trajectory_status_.push_back(1);
            }
            else
            {
              smoothing_trajectory_status_.push_back(2);
            }

            enu_smoothing_trajectory_east_.push_back(trajectory_ef[j]*(1-ratio[j]) + trajectory_er[j]*ratio[j]);
            enu_smoothing_trajectory_north_.push_back(trajectory_nf[j]*(1-ratio[j]) + trajectory_nr[j]*ratio[j]);
          }

          trajectory_ef.clear();
          trajectory_er.clear();
          trajectory_nf.clear();
          trajectory_nr.clear();
          diff_time.clear();
          tmp_vel.clear();
          all_vel.clear();
          ratio.clear();
        }

        for(i = index_gga[0]; i < index_gga[index_gga_length-1]; i++)
        {
          tmp_llh[0] = gga_[i].lat *M_PI/180;
          tmp_llh[1] = gga_[i].lon*M_PI/180;
          tmp_llh[2] = gga_[i].alt + gga_[i].undulation;

          llh2xyz(tmp_llh, tmp_ecef);
          xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu);

          enu_smoothing_trajectory_height_.push_back(tmp_enu[2]);
        }

        tmp_llh[0] = gga_[index_gga[index_gga_length-1]].lat *M_PI/180;
        tmp_llh[1] = gga_[index_gga[index_gga_length-1]].lon*M_PI/180;
        tmp_llh[2] = gga_[index_gga[index_gga_length-1]].alt + gga_[index_gga[index_gga_length-1]].undulation;

        llh2xyz(tmp_llh, tmp_ecef);
        xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu);

        for(i = index_gga[index_gga_length-1]; i < gga_length; i++)
        {
          int index;

          if(time_last != gga_[i].header.stamp.toSec() &&gga_[i].gps_qual ==4)
          {
            index = i;
            time_last = gga_[i].header.stamp.toSec();
            tmp_llh[0] = gga_[i].lat *M_PI/180;
            tmp_llh[1] = gga_[i].lon*M_PI/180;
            tmp_llh[2] = gga_[i].alt + gga_[i].undulation;

            llh2xyz(tmp_llh, tmp_ecef);
            xyz2enu(tmp_ecef, tmp_ecef_base, tmp_enu);
          }

          smoothing_trajectory_status_.push_back(2);
          enu_smoothing_trajectory_east_.push_back(eagleye_state_forward_.enu_relative_pos[i].enu_pos.x + tmp_enu[0] -
            eagleye_state_forward_.enu_relative_pos[index].enu_pos.x);
          enu_smoothing_trajectory_north_.push_back(eagleye_state_forward_.enu_relative_pos[i].enu_pos.y + tmp_enu[1] -
            eagleye_state_forward_.enu_relative_pos[index].enu_pos.y);
          enu_smoothing_trajectory_height_.push_back(tmp_enu[2]);
        }
      }
      else
      {
        for(i = 0; i < gga_length; i++)
        {
          smoothing_trajectory_status_.push_back(-1);
          enu_smoothing_trajectory_east_.push_back(0);
          enu_smoothing_trajectory_north_.push_back(0);
          enu_smoothing_trajectory_height_.push_back(0);
        }
      }

      enu_smoothing_trajectory_length = std::distance(enu_smoothing_trajectory_east_.begin(), enu_smoothing_trajectory_east_.end());

      for(i = 0; i < enu_smoothing_trajectory_length; i++)
      {

        enu_smoothing_trajectory[0] = enu_smoothing_trajectory_east_[i];
        enu_smoothing_trajectory[1] = enu_smoothing_trajectory_north_[i];
        enu_smoothing_trajectory[2] = enu_smoothing_trajectory_height_[i];

        enu2llh(enu_smoothing_trajectory, tmp_ecef_base, llh_smoothing_trajectory);

        llh_smoothing_trajectory_lat_.push_back(llh_smoothing_trajectory[0] * 180/M_PI);
        llh_smoothing_trajectory_lon_.push_back(llh_smoothing_trajectory[1] * 180/M_PI);
        llh_smoothing_trajectory_hei_.push_back(llh_smoothing_trajectory[2]);
      }
    }
  }
}


/*****************************
convertHeight
Function to convert height
******************************/
void eagleye_pp::convertHeight(void)
{
 ConvertHeight convert_height;
 if (convert_height_num_ == 1)
  {
    for(int i = 0; i < data_length_; i++)
    {
      convert_height.setLLH(rtklib_nav_[i].status.latitude, rtklib_nav_[i].status.longitude, rtklib_nav_[i].status.altitude);
      rtklib_nav_[i].status.altitude = convert_height.convert2altitude();
      convert_height.setLLH(eagleye_state_forward_.eagleye_fix[i].latitude, eagleye_state_forward_.eagleye_fix[i].longitude, eagleye_state_forward_.eagleye_fix[i].altitude);
      eagleye_state_forward_.eagleye_fix[i].altitude = convert_height.convert2altitude();
      convert_height.setLLH(eagleye_state_backward_.eagleye_fix[i].latitude, eagleye_state_backward_.eagleye_fix[i].longitude, eagleye_state_backward_.eagleye_fix[i].altitude);
      eagleye_state_backward_.eagleye_fix[i].altitude = convert_height.convert2altitude();
      convert_height.setLLH(llh_smoothing_trajectory_lat_[i], llh_smoothing_trajectory_lon_[i], llh_smoothing_trajectory_hei_[i]);
      llh_smoothing_trajectory_hei_[i] = convert_height.convert2altitude();
    }
  }
  else if(convert_height_num_ == 2)
  {
    for(int i = 0; i < data_length_; i++)
    {
      convert_height.setLLH(rtklib_nav_[i].status.latitude, rtklib_nav_[i].status.longitude, rtklib_nav_[i].status.altitude);
      rtklib_nav_[i].status.altitude = convert_height.convert2ellipsoid();
      convert_height.setLLH(eagleye_state_forward_.eagleye_fix[i].latitude, eagleye_state_forward_.eagleye_fix[i].longitude, eagleye_state_forward_.eagleye_fix[i].altitude);
      eagleye_state_forward_.eagleye_fix[i].altitude = convert_height.convert2ellipsoid();
      convert_height.setLLH(eagleye_state_backward_.eagleye_fix[i].latitude, eagleye_state_backward_.eagleye_fix[i].longitude, eagleye_state_backward_.eagleye_fix[i].altitude);
      eagleye_state_backward_.eagleye_fix[i].altitude = convert_height.convert2ellipsoid();
      convert_height.setLLH(llh_smoothing_trajectory_lat_[i], llh_smoothing_trajectory_lon_[i], llh_smoothing_trajectory_hei_[i]);
      llh_smoothing_trajectory_hei_[i] = convert_height.convert2ellipsoid();
    }
  }
}

void eagleye_pp::writePointKMLOneWay(std::stringstream* eagleye_plot, std::string name,const EagleyeStates& eagleye_state, int i)
{
  *eagleye_plot << "\t\t<Placemark>\n\
  \t\t\t<name>" << name << "</name>\n\
  \t\t\t<visibility>0</visibility>\n\
  \t\t\t<Snippet></Snippet>\n\
  \t\t\t<description><![CDATA[<B>Eagleye Status</B><BR><BR>\n\
  \t\t\t\t<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n\
  \t\t\t\t\t<TR ALIGN=RIGHT>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Sequence Number: </TD><TD>";
  *eagleye_plot << i;
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Timestamp:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << imu_[i].header.stamp.toNSec();
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>CAN Velocity:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.x * 3.6 << " [km/h]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>IMU Linear Acceleration X:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.x << " [m/s^2]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>IMU Linear Acceleration Y:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.y << " [m/s^2]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>IMU Linear Acceleration Z:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.z << " [m/s^2]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>IMU Angular Velocity X:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.x << " [rad/s]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>IMU Angular Velocity Y:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.y << " [rad/s]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>IMU Angular Velocity Z:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.z << " [rad/s]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Velocity:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.velocity_scale_factor[i].correction_velocity.linear.x * 3.6 << " [km/h]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Velocity Scale Factor:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.velocity_scale_factor[i].scale_factor;
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Velocity Scale Factor Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.velocity_scale_factor[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Heading 1st:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_1st[i].heading_angle << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Heading 1st Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.heading_interpolate_1st[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Heading 2nd:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_2nd[i].heading_angle << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Heading 2nd Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.heading_interpolate_2nd[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Heading 3rd:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.heading_interpolate_3rd[i].heading_angle << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Heading 3rd Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.heading_interpolate_3rd[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Yawrate Offset Stop:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.yawrate_offset_stop[i].yawrate_offset << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Yawrate Offset Stop Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.yawrate_offset_stop[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Yawrate Offset 1st:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.yawrate_offset_1st[i].yawrate_offset << " [rad]";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.yawrate_offset_2nd[i].yawrate_offset << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Yawrate Offset 2nd Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.yawrate_offset_2nd[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Tire Slip Angle:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.slip_angle[i].slip_angle << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Tire Slip Angle Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.slip_angle[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Pitchi Angle:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.pitching[i].pitching_angle << " [rad]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Pitchi Angle Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.pitching[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Height Flag:</TD><TD>";
  *eagleye_plot << bool(eagleye_state.height[i].status.enabled_status);
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Latitude:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].latitude << " [deg]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Longitude:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].longitude << " [deg]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Estimate Altitude:</TD><TD>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].altitude << " [m]";
  *eagleye_plot << "</TD></TR>\n\
  \t\t\t\t</TABLE>\n\
  \t\t\t]]></description>\n\
  \t\t\t<styleUrl>";
  if(bool(eagleye_state.enu_absolute_pos_interpolate[i].status.estimate_status) == false)
  {
    *eagleye_plot << "#EAGLEYE";
  }
  else
  {
    *eagleye_plot << "#EAGLEYE_EST";
  }
  *eagleye_plot << "</styleUrl>\n\
  \t\t\t<Point>\n\
  \t\t\t\t<coordinates>";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].longitude << ",";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].latitude << ",";
  *eagleye_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state.eagleye_fix[i].altitude << ",";
  *eagleye_plot << "</coordinates>\n\
  \t\t\t</Point>\n\
  \t\t</Placemark>\
  " << std::endl;
}

/******************************************************************************
writePointKML
Function to output kml(plot)
bool arg_use_rtk_navsatfix_topic : Determine if to use navsatfix
std::string* arg_s_eagleye_line : forward line data
std::string* arg_s_eagleye_back_line : backward line data
std::string* arg_s_eagleye_pp_line : Line data to merge forward and backward
******************************************************************************/
void eagleye_pp::writePointKML(bool arg_use_rtk_navsatfix_topic,std::string* arg_s_eagleye_line,std::string* arg_s_eagleye_back_line, std::string* arg_s_eagleye_pp_line)
{
  std::ofstream output_plot_kml_file(outputpath_ + "eagleye.kml", std::ios::out);
  std::cout << "Output file = " << outputpath_ << "eagleye.kml" << std::endl;

  std::stringstream eagleye_plot;
  std::stringstream eagleye_back_plot;
  std::stringstream rtklib_plot;
  std::stringstream gnss_plot;
  std::stringstream eagleye_pp_plot;

  int rtknav_seq_last = 0, gga_seq_last = 0;
  double driving_distance = 0, driving_distance_last = 0, driving_distance_last2 = 0, driving_distance_last3 = 0,
    driving_distance_last4 = 0, driving_distance_last5 = 0;

  if (!boost::filesystem::is_directory(outputpath_))
  {
    boost::filesystem::create_directory(outputpath_);
    std::cout << "Directory created: " << outputpath_ << std::endl;
  }

  output_plot_kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
<kml xmlns=\"http://earth.google.com/kml/2.2\">\n\
<Document>\n\
<name>";
output_plot_kml_file << "eagleye";
output_plot_kml_file << "</name>\n\
<open>1</open>\n\
<Folder id=\"ID1\">\n\
<name>Screen Overlays</name>\n\
<visibility>0</visibility>\n\
<open>0</open>\n\
\t<ScreenOverlay>\n\
\t\t<name>Eagleye Logo</name>\n\
\t\t<visibility>1</visibility>\n\
\t\t<Icon>\n\
\t\t\t<href>https://github.com/MapIV/eagleye/blob/main-ros1/docs/logo.png?raw=true</href>\n\
\t\t</Icon>\n\
\t\t<overlayXY x=\"-0.3\" y=\"-1\" xunits=\"fraction\" yunits=\"fraction\"/>\n\
\t\t<screenXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n\
\t\t<size x=\"260.6\" y=\"56.0\" xunits=\"pixels\" yunits=\"pixels\"/>\n\
\t</ScreenOverlay>\n\
</Folder>\n\
\t<Style id=\"EAGLEYE\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ff0000ff</color>\n\
\t\t\t<scale>0.5</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_EST\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ff0000ff</color>\n\
\t\t\t<scale>0.4</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/square.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"RTKLIB\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ffff00ff</color>\n\
\t\t\t<scale>0.5</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"RTKLIB_FIX\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ffff00ff</color>\n\
\t\t\t<scale>0.4</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/square.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"GNSS\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ff00ffff</color>\n\
\t\t\t<scale>0.5</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"GNSS_FIX\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ff00ff00</color>\n\
\t\t\t<scale>0.4</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/square.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_PP_STATUS_0\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ff00ff00</color>\n\
\t\t\t<scale>0.5</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.googros/ros.hle.com/mapfiles/kml/shapes/placemark_circle.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_PP_STATUS_1\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ffff00ff</color>\n\
\t\t\t<scale>0.5</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_PP_STATUS_2\">\n\
\t\t<IconStyle>\n\
\t\t\t<color>ff0000ff</color>\n\
\t\t\t<scale>0.5</scale>\n\
\t\t\t<Icon>\n\
\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n\
\t\t\t</Icon>\n\
\t\t</IconStyle>\n\
\t\t<LabelStyle>\n\
\t\t\t<scale>0</scale>\n\
\t\t</LabelStyle>\n\
\t\t<BalloonStyle>\n\
\t\t\t<bgColor>fffff8f0</bgColor>\n\
\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n\
\t\t</BalloonStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_LINE\">\n\
\t\t<LineStyle>\n\
\t\t\t<color>ff0000ff</color>\n\
\t\t\t<width>5.00</width>\n\
\t\t</LineStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_BACK_LINE\">\n\
\t\t<LineStyle>\n\
\t\t\t<color>ff00ff00</color>\n\
\t\t\t<width>5.00</width>\n\
\t\t</LineStyle>\n\
\t</Style>\n\
\t<Style id=\"EAGLEYE_PP_LINE\">\n\
\t\t<LineStyle>\n\
\t\t\t<color>ffff0000</color>\n\
\t\t\t<width>5.00</width>\n\
\t\t</LineStyle>\n\
\t</Style>\n\
<Folder id=\"ID2\">\n\
\t<name>CAR Trajectry</name>\n\
\t<visibility>1</visibility>\n\
\t<open>0</open>\
" << std::endl;

for(int i = 0; i < data_length_; i++)
  {
    if (output_kml_eagleye_forward_plot_)
    {
      if(bool(eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.enabled_status) == true)
      {
        if(std::abs((eagleye_state_forward_.distance[i].distance - driving_distance_last)) > interval_plot_ ||
          bool(eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.estimate_status) == true)
        {

          writePointKMLOneWay(&eagleye_plot, "Eagleye", eagleye_state_forward_, i);

          driving_distance_last = eagleye_state_forward_.distance[i].distance;
        }
      }
    }

    if(output_kml_eagleye_pp_plot_)
    {
      if(smoothing_trajectory_status_[i] != -1)
      {
        eagleye_pp_plot << "\t<Placemark>\n\
        \t\t<name>Eagleye_pp</name>\n\
        \t\t<visibility>0</visibility>\n\
        \t\t<Snippet></Snippet>\n\
        \t\t<styleUrl>";
        if(smoothing_trajectory_status_[i] == 0)
        {
          eagleye_pp_plot << "#EAGLEYE_PP_STATUS_0";
        }
        else if(smoothing_trajectory_status_[i] == 1)
        {
          eagleye_pp_plot << "#EAGLEYE_PP_STATUS_1";
        }
        else if(smoothing_trajectory_status_[i] == 2)
        {
          eagleye_pp_plot << "#EAGLEYE_PP_STATUS_2";
        }
        eagleye_pp_plot << "</styleUrl>\n\
        \t\t<Point>\n\
        \t\t\t<coordinates>";
        eagleye_pp_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lon_[i] << ",";
        eagleye_pp_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lat_[i] << ",";
        eagleye_pp_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_hei_[i] << ",";
        eagleye_pp_plot << "</coordinates>\n\
        \t\t</Point>\n\
        \t</Placemark>\
        " << std::endl;
      }
    }

    if (output_kml_eagleye_backward_plot_)
    {
      if(bool(eagleye_state_backward_.enu_absolute_pos_interpolate[i].status.enabled_status) == true)
      {
        if(std::abs((eagleye_state_backward_.distance[i].distance - driving_distance_last4)) > interval_plot_ ||
        bool(eagleye_state_backward_.enu_absolute_pos_interpolate[i].status.estimate_status) == true)
        {
          writePointKMLOneWay(&eagleye_back_plot, "Eagleye_Back", eagleye_state_backward_, i);

          driving_distance_last4 = eagleye_state_backward_.distance[i].distance;
        }
      }
    }

    if (rtklib_nav_[i].header.seq - rtknav_seq_last != 0 && output_kml_rtklib_plot_)
    {
      rtklib_plot << "\t<Placemark>\n\
      \t\t<name>RTKLIB</name>\n\
      \t\t<visibility>0</visibility>\n\
      \t\t<Snippet></Snippet>\n\
      <description><![CDATA[<B>RTKLIB Status</B><BR><BR>\n\
      \t\t\t<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n\
      \t\t\t\t<TR ALIGN=RIGHT>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Sequence Number: </TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].header.seq;
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Timestamp:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].header.stamp.toNSec();
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time of Week:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].tow << " [ms]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>ECEF X:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.x << " [m]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>ECEF Y:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.y << " [m]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>ECEF Z:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.z << " [m]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>ECEF Velocity X:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.x << " [m/s]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>ECEF Velocity Y:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.y << " [m/s]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>ECEF Velocity Z:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.z << " [m/s]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Status:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << int(rtklib_nav_[i].status.status.status);
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Service:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].status.status.service;
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Latitude:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.latitude << " [deg]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Longitude:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.longitude << " [deg]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Altitude:</TD><TD>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.altitude << " [m]";
      rtklib_plot << "</TD></TR>\n\
      \t\t\t</TABLE>\n\
      ]]></description>\n\
      \t\t<styleUrl>";
      if(rtklib_nav_[i].status.status.status == -1)
      {
        rtklib_plot << "#RTKLIB";
      }
      else
      {
        rtklib_plot << "#RTKLIB_FIX";
      }
      rtklib_plot << "</styleUrl>\n\
      \t\t<Point>\n\
      \t\t\t<coordinates>";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.longitude << ",";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.latitude << ",";
      rtklib_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.altitude << ",";
      rtklib_plot << "</coordinates>\n\
      \t\t</Point>\n\
      \t</Placemark>\
      " << std::endl;
      rtknav_seq_last = rtklib_nav_[i].header.seq;
    }

    if (gga_[i].header.seq - gga_seq_last != 0 && output_kml_gnss_plot_)
    {
      gnss_plot << "\t<Placemark>\n\
      \t\t<name>GNSS Receiver</name>\n\
      \t\t<visibility>0</visibility>\n\
      \t\t<Snippet></Snippet>\n\
      <description><![CDATA[<B>GNSS Receiver Status</B><BR><BR>\n\
      \t\t\t<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n\
      \t\t\t\t<TR ALIGN=RIGHT>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Sequence Number: </TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_[i].header.seq;
      gnss_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Timestamp:</TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_[i].header.stamp.toNSec();
      gnss_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Status:</TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << int(gga_[i].gps_qual);
      gnss_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Service:</TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<int>::max_digits10) << 0;
      gnss_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Latitude:</TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lat << " [deg]";
      gnss_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Longitude:</TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lon << " [deg]";
      gnss_plot << "</TD></TR>\n\
      \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Altitude:</TD><TD>";
      gnss_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].alt +  gga_[i].undulation << " [m]";
      gnss_plot << "</TD></TR>\n\
      \t\t\t</TABLE>\n\
      ]]></description>\n\
      \t\t<styleUrl>";
      if(gga_[i].gps_qual != 4)
      {
        gnss_plot << "#GNSS";
      }
      else
      {
        gnss_plot << "#GNSS_FIX";
      }
      gnss_plot << "</styleUrl>\n\
      \t\t<Point>\n\
      \t\t\t<coordinates>";
      gnss_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lon << ",";
      gnss_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lat << ",";
      gnss_plot << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].alt +  gga_[i].undulation << ",";
      gnss_plot << "</coordinates>\n\
      \t\t</Point>\n\
      \t</Placemark>\
      " << std::endl;
      gga_seq_last = gga_[i].header.seq;
    }
  }

  std::string s_eagleye_plot = eagleye_plot.str();
  std::string s_eagleye_back_plot = eagleye_back_plot.str();
  std::string s_rtklib_plot = rtklib_plot.str();
  std::string s_gnss_plot = gnss_plot.str();
  std::string s_eagleye_pp_plot = eagleye_pp_plot.str();

  if(output_kml_eagleye_forward_plot_)
  {

    output_plot_kml_file <<"\t<Folder id=\"ID21\">\n\
    \t<name>EAGLEYE_FORWARD</name>\n\
    \t<visibility>0</visibility>\n\
    \t<open>0</open>\n\
    " << s_eagleye_plot <<"\t</Folder>\n";
  }

  if(output_kml_eagleye_backward_plot_)
  {
    output_plot_kml_file <<"\t<Folder id=\"ID21\">\n\
    \t<name>EAGLEYE_BACKWARD</name>\n\
    \t<visibility>0</visibility>\n\
    \t<open>0</open>\n\
    " << s_eagleye_back_plot <<"\t</Folder>\n";
  }

  if(output_kml_eagleye_pp_plot_)
  {
    output_plot_kml_file <<"\t<Folder id=\"ID24\">\n\
    \t<name>EAGLEYE_PP</name>\n\
    \t<visibility>0</visibility>\n\
    \t<open>0</open>\n\
    " << s_eagleye_pp_plot <<"\t</Folder>\n";
  }

  if(output_kml_rtklib_plot_)
  {
    output_plot_kml_file <<"\t<Folder id=\"ID22\">\n\
    \t<name>RTKLIB</name>\n\
    \t<visibility>0</visibility>\n\
    \t<open>0</open>\n\
    " << s_rtklib_plot <<"\t</Folder>\n";
  }

  if(output_kml_gnss_plot_)
  {
    output_plot_kml_file <<"\t<Folder id=\"ID23\">\n\
    \t<name>GNSS</name>\n\
    \t<visibility>0</visibility>\n\
    \t<open>0</open>\n\
    " << s_gnss_plot <<"\t</Folder>\n";
  }

  if(output_kml_eagleye_forward_line_)
  {
      output_plot_kml_file <<"\t<Placemark>\n\
    \t<name>EAGLEYE_FORWARD_LINE</name>\n\
    \t<visibility>" << (!arg_use_rtk_navsatfix_topic ? "1" : "0") <<"</visibility>\n\
    \t<description><![CDATA[]]></description>\n\
    \t<styleUrl>#EAGLEYE_LINE</styleUrl>\n\
    \t<LineString>\n\
    \t\t<extrude>0</extrude>\n\
    \t\t<tessellate>1</tessellate>\n\
    \t\t<altitudeMode>clampToGround</altitudeMode>\n\
    \t\t<coordinates>\n\
    " << *arg_s_eagleye_line <<"\t\t</coordinates>\n\
    \t</LineString>\n\
    \t</Placemark>\n";
  }
 if(output_kml_eagleye_backward_line_)
  {

    output_plot_kml_file <<"\t<Placemark>\n\
    \t<name>EAGLEYE_BACKWARD_LINE</name>\n\
    \t<visibility>0</visibility>\n\
    \t<description><![CDATA[]]></description>\n\
    \t<styleUrl>#EAGLEYE_BACK_LINE</styleUrl>\n\
    \t<LineString>\n\
    \t\t<extrude>0</extrude>\n\
    \t\t<tessellate>1</tessellate>\n\
    \t\t<altitudeMode>clampToGround</altitudeMode>\n\
    \t\t<coordinates>\n\
    " << *arg_s_eagleye_back_line <<"\t\t</coordinates>\n\
    \t</LineString>\n\
    \t</Placemark>\n";
  }
  if(output_kml_eagleye_pp_line_)
  {
    output_plot_kml_file <<"\t<Placemark>\n\
    \t<name>EAGLEYE_PP_LINE</name>\n\
    \t<visibility>" << (arg_use_rtk_navsatfix_topic ? "1" : "0") <<"</visibility>\n\
    \t<description><![CDATA[]]></description>\n\
    \t<styleUrl>#EAGLEYE_PP_LINE</styleUrl>\n\
    \t<LineString>\n\
    \t\t<extrude>0</extrude>\n\
    \t\t<tessellate>1</tessellate>\n\
    \t\t<altitudeMode>clampToGround</altitudeMode>\n\
    \t\t<coordinates>\n\
    " << *arg_s_eagleye_pp_line <<"\t\t</coordinates>\n\
    \t</LineString>\n\
    \t</Placemark>\n";
  }

  output_plot_kml_file <<"</Folder>\n\
  </Document>\n\
  </kml>\
  " << std::endl;
}

void eagleye_pp::addHeaderLineKML(std::ofstream* output_line_kml_file)
{
  *output_line_kml_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
  <kml xmlns=\"http://earth.google.com/kml/2.2\">\n\
  <Document>\n\
  <name>\n\
  " << std::endl ;
  *output_line_kml_file << "eagleye" << std::endl;
  *output_line_kml_file << "</name>\n\
  <ScreenOverlay>\n\
  \t<name>Eagleye Logo</name>\n\
  \t<visibility>1</visibility>\n\
  \t<Icon>\n\
  \t\t<href>https://github.com/MapIV/eagleye/blob/main-ros1/docs/logo.png?raw=true</href>\n\
  \t</Icon>\n\
  \t<overlayXY x=\"-0.3\" y=\"-1\" xunits=\"fraction\" yunits=\"fraction\"/>\n\
  \t<screenXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n\
  \t<size x=\"260.6\" y=\"56.0\" xunits=\"pixels\" yunits=\"pixels\"/>\n\
  </ScreenOverlay>\n\
  " << std::endl;
  return;
}

void eagleye_pp::addLineStyleInKML(std::ofstream* output_line_kml_file, std::string style_id, std::string color)
{
  *output_line_kml_file << "<Style id=\"" << style_id << "\">\n\
  \t<LineStyle>\n\
  \t\t<color>" << color << "</color>\n\
  \t\t<width>5.00</width>\n\
  \t</LineStyle>\n\
  </Style>\n\
  " << std::endl;
}

void eagleye_pp::addPlacemarkInKML(std::ofstream* output_line_kml_file,std::string name, int visibility,std::string coordinates)
{
    *output_line_kml_file <<"\t<Placemark>\n\
    \t<name>" << name << "</name>\n\
    \t<visibility>" << visibility <<"</visibility>\n\
    \t<description><![CDATA[]]></description>\n\
    \t<styleUrl>#" << name << "</styleUrl>\n\
    \t<LineString>\n\
    \t\t<extrude>0</extrude>\n\
    \t\t<tessellate>1</tessellate>\n\
    \t\t<altitudeMode>clampToGround</altitudeMode>\n\
    \t\t<coordinates>\n\
    " << coordinates <<"\t\t</coordinates>\n\
    \t</LineString>\n\
    \t</Placemark>\n";
}

void eagleye_pp::updateLineKML(std::stringstream* eagleye_line,double llh[3], eagleye_msgs::Distance distance, double * driving_distance_last)
{

  if(std::abs((distance.distance - *driving_distance_last)) > interval_line_)
  {
    *eagleye_line << std::setprecision(std::numeric_limits<double>::max_digits10) << llh[0] << "," << llh[1] << "," << llh[2] << std::endl;
    *driving_distance_last = distance.distance;
  }

  return;
}


/************************************************************************************************
writeLineKML
Function to output kml(line)
bool arg_use_rtk_navsatfix_topic : Determine if to use navsatfix
std::string* arg_s_eagleye_line : forward line data(output)
std::string* arg_s_eagleye_back_line : backward line data(output)
std::string* arg_s_eagleye_pp_line : Line data to merge forward and backward(output)
*************************************************************************************************/
void eagleye_pp::writeLineKML(bool arg_use_rtk_navsatfix_topic, std::string* arg_s_eagleye_line, std::string* arg_s_eagleye_back_line, std::string* arg_s_eagleye_pp_line)
{
  std::ofstream output_line_kml_file(outputpath_ + "eagleye_line.kml", std::ios::out);
  std::cout << "Output file = " << outputpath_ << "eagleye_line.kml" << std::endl;
  if(!arg_use_rtk_navsatfix_topic)
  {
    std::cout << "\033[1;33mWarn: Display kml of forward processing results.\033[0m" << std::endl;
  }

  std::stringstream eagleye_line;
  std::stringstream eagleye_back_line;
  std::stringstream eagleye_pp_line;

  double driving_distance = 0,driving_distance_last = 0,driving_distance_last2 = 0,
    driving_distance_last3 = 0,driving_distance_last4 = 0,driving_distance_last5 = 0;

  if(!boost::filesystem::is_directory(outputpath_))
  {
    boost::filesystem::create_directory(outputpath_);
    std::cout << "Directory created: " << outputpath_ << std::endl;
  }

  addHeaderLineKML(&output_line_kml_file);

  addLineStyleInKML(&output_line_kml_file, "EAGLEYE_LINE", "ff0000ff");
  addLineStyleInKML(&output_line_kml_file, "EAGLEYE_BACK_LINE", "ff00ff00");
  addLineStyleInKML(&output_line_kml_file, "EAGLEYE_PP_LINE", "ffff0000");

  for(int i = 0; i<data_length_; i++)
  {
    if (bool(eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.enabled_status) == true)
    {
      if(std::abs((eagleye_state_forward_.distance[i].distance - driving_distance_last)) > interval_plot_ ||
        bool(eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.estimate_status) == true)
      {
        double llh[3] = {eagleye_state_forward_.eagleye_fix[i].longitude, eagleye_state_forward_.eagleye_fix[i].latitude,
          eagleye_state_forward_.eagleye_fix[i].altitude};
        updateLineKML(&eagleye_line,llh,eagleye_state_forward_.distance[i], &driving_distance_last2);
        driving_distance_last = eagleye_state_forward_.distance[i].distance;
      }
    }
  }

  if(output_kml_eagleye_pp_line_)
  {
    for(int i = 0; i<data_length_; i++)
    {
      if(smoothing_trajectory_status_[i] != -1)
      {
        double llh[3] = {llh_smoothing_trajectory_lon_[i], llh_smoothing_trajectory_lat_[i], llh_smoothing_trajectory_hei_[i]};
        updateLineKML(&eagleye_pp_line,llh,eagleye_state_forward_.distance[i], &driving_distance_last3);
      }
    }
  }

  if(output_kml_eagleye_backward_line_)
  {
    for(int i = 0; i<data_length_; i++)
    {
      if (bool(eagleye_state_backward_.enu_absolute_pos_interpolate[i].status.enabled_status) == true)
      {
        if(std::abs((eagleye_state_backward_.distance[i].distance - driving_distance_last4)) > interval_plot_ ||
          bool(eagleye_state_backward_.enu_absolute_pos_interpolate[i].status.estimate_status) == true)
        {
          double llh[3] = {eagleye_state_backward_.eagleye_fix[i].longitude, eagleye_state_backward_.eagleye_fix[i].latitude,
            eagleye_state_backward_.eagleye_fix[i].altitude};
          updateLineKML(&eagleye_back_line,llh,eagleye_state_backward_.distance[i], &driving_distance_last5);

          driving_distance_last4 = eagleye_state_backward_.distance[i].distance;
        }
      }
    }
  }

  *arg_s_eagleye_line = eagleye_line.str();
  *arg_s_eagleye_back_line = eagleye_back_line.str();
  *arg_s_eagleye_pp_line = eagleye_pp_line.str();

  if(output_kml_eagleye_forward_line_)
  {
    int visibility = (!arg_use_rtk_navsatfix_topic ? 1 : 0);
    addPlacemarkInKML(&output_line_kml_file, "EAGLEYE_LINE", visibility, *arg_s_eagleye_line);
  }

  if(output_kml_eagleye_backward_line_)
  {
    addPlacemarkInKML(&output_line_kml_file, "EAGLEYE_BACK_LINE", 0, *arg_s_eagleye_back_line);
  }

  if(output_kml_eagleye_pp_line_)
  {
    int visibility = (!arg_use_rtk_navsatfix_topic ? 1 : 0);
    addPlacemarkInKML(&output_line_kml_file, "EAGLEYE_PP_LINE", visibility, *arg_s_eagleye_pp_line);
  }

  output_line_kml_file << "</Document>\n\
  </kml>\n\
  " << std::endl;
}


/************************************************************
writeSimpleCSV
Function to output csv(Simplified version)
************************************************************/
void eagleye_pp::writeSimpleCSV(void)
{
  std::ofstream output_csv_file(outputpath_ + "eagleye.csv", std::ios::out);
  std::cout << "Output file = " << outputpath_ << "eagleye.csv" << std::endl;

  if (!boost::filesystem::is_directory(outputpath_))
  {
    boost::filesystem::create_directory(outputpath_);
    std::cout << "Directory created: " << outputpath_ << std::endl;
  }

  // output_csv_file header
output_csv_file << "timestamp,eagleye_llh.latitude,eagleye_llh.longitude,eagleye_llh.altitude\
,eagleye_llh.orientation_covariance[0],eagleye_llh.orientation_covariance[1],eagleye_llh.orientation_covariance[2],eagleye_llh.orientation_covariance[3],eagleye_llh.orientation_covariance[4],eagleye_llh.orientation_covariance[5],eagleye_llh.orientation_covariance[6],eagleye_llh.orientation_covariance[7],eagleye_llh.orientation_covariance[8]\
,eagleye_llh.status,eagleye_twist.linear.x,eagleye_twist.linear.y,eagleye_twist.linear.z,eagleye_twist.angular.x,eagleye_twist.angular.y,eagleye_twist.angular.z\
,eagleye_twist.orientation_covariance[0],eagleye_twist.orientation_covariance[1],eagleye_twist.orientation_covariance[2],eagleye_twist.orientation_covariance[3],eagleye_twist.orientation_covariance[4],eagleye_twist.orientation_covariance[5],eagleye_twist.orientation_covariance[6],eagleye_twist.orientation_covariance[7],eagleye_twist.orientation_covariance[8]\
,eagleye_twist.orientation_covariance[9],eagleye_twist.orientation_covariance[10],eagleye_twist.orientation_covariance[11],eagleye_twist.orientation_covariance[12],eagleye_twist.orientation_covariance[13],eagleye_twist.orientation_covariance[14],eagleye_twist.orientation_covariance[15],eagleye_twist.orientation_covariance[16],eagleye_twist.orientation_covariance[17]\
,eagleye_acceleration.x,eagleye_acceleration.y,eagleye_acceleration.z\
,eagleye_acceleration.orientation_covariance[0],eagleye_acceleration.orientation_covariance[1],eagleye_acceleration.orientation_covariance[2],eagleye_acceleration.orientation_covariance[3],eagleye_acceleration.orientation_covariance[4],eagleye_acceleration.orientation_covariance[5],eagleye_acceleration.orientation_covariance[6],eagleye_acceleration.orientation_covariance[7],eagleye_acceleration.orientation_covariance[8]\
,eagleye_posture.roll,eagleye_posture.pitch,eagleye_posture.yaw\
,eagleye_posture.orientation_covariance[0],eagleye_posture.orientation_covariance[1],eagleye_posture.orientation_covariance[2],eagleye_posture.orientation_covariance[3],eagleye_posture.orientation_covariance[4],eagleye_posture.orientation_covariance[5],eagleye_posture.orientation_covariance[6],eagleye_posture.orientation_covariance[7],eagleye_posture.orientation_covariance[8]\
,navsat_llh.latitude,navsat_llh.longitude,navsat_llh.altitude\
,navsat_llh.orientation_covariance[0],navsat_llh.orientation_covariance[1],navsat_llh.orientation_covariance[2],navsat_llh.orientation_covariance[3],navsat_llh.orientation_covariance[4],navsat_llh.orientation_covariance[5],navsat_llh.orientation_covariance[6],navsat_llh.orientation_covariance[7],navsat_llh.orientation_covariance[8]\
,navsat_llh.gps_qual\
,eagleye_pp_llh.latitude,eagleye_pp_llh.longitude,eagleye_pp_llh.altitude\
,eagleye_pp_llh.orientation_covariance[0],eagleye_pp_llh.orientation_covariance[1],eagleye_pp_llh.orientation_covariance[2],eagleye_pp_llh.orientation_covariance[3],eagleye_pp_llh.orientation_covariance[4],eagleye_pp_llh.orientation_covariance[5],eagleye_pp_llh.orientation_covariance[6],eagleye_pp_llh.orientation_covariance[7],eagleye_pp_llh.orientation_covariance[8]\
,eagleye_pp_llh.status\
" << std::endl;

  //output debug file
  for(int i=0; i<data_length_; i++)
  {
    output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << imu_[i].header.stamp.toNSec() << ","; //timestamp
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].latitude << ","; //eagleye_llh.latitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].longitude << ","; //eagleye_llh.longitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].altitude << ","; //eagleye_llh.altitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[0] << ","; //eagleye_llh.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[1] << ","; //eagleye_llh.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[2] << ","; //eagleye_llh.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[3] << ","; //eagleye_llh.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[4] << ","; //eagleye_llh.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[5] << ","; //eagleye_llh.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[6] << ","; //eagleye_llh.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[7] << ","; //eagleye_llh.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_fix[i].position_covariance[8] << ","; //eagleye_llh.orientation_covariance[8]
    output_csv_file << bool(eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.estimate_status) << ","; //eagleye_llh.status
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.linear.x << ","; //eagleye_twist.linear.x
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.linear.y << ","; //eagleye_twist.linear.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.linear.z << ","; //eagleye_twist.linear.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.angular.x << ","; //eagleye_twist.angular.x
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.angular.y << ","; //eagleye_twist.angular.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.eagleye_twist[i].twist.angular.z << ","; //eagleye_twist.angular.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[9]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[10]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[11]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[12]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[13]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[14]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[15]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[16]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_twist.orientation_covariance[17]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.x << ","; //eagleye_acceleration.x TODO Change to acceleration estimated by eagleye
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.y << ","; //eagleye_acceleration.y
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.z << ","; //eagleye_acceleration.z
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_acceleration.orientation_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.rolling[i].rolling_angle << ","; //eagleye_posture.roll
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.pitching[i].pitching_angle << ","; //eagleye_posture.pitch
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_3rd[i].heading_angle << ","; //eagleye_posture.yaw
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_posture.orientation_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lat << ","; //navsat_llh.latitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lon << ","; //navsat_llh.longitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].alt +  gga_[i].undulation<< ","; //navsat_llh.altitude
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //navsat_llh.orientation_covariance[8]
    output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(gga_[i].gps_qual) << ","; //navsat_llh.gps_qual
    if(getUseCombination())
    {
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lat_[i] << ","; //eagleye_pp_llh.latitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lon_[i] << ","; //eagleye_pp_llh.longitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_hei_[i] << ","; //eagleye_pp_llh.altitude
    }
    else
    {
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.latitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.longitude
      output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.altitude      
    }
    
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[0]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[1]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[2]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[3]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[4]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[5]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[6]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[7]
    output_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[8]
    if(getUseCombination())
    {
      output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << smoothing_trajectory_status_[i] << ","; //eagleye_pp_llh.status
    }
    else
    {
      output_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //eagleye_pp_llh.status    
    }
    output_csv_file << "\n";
  }
}


void eagleye_pp::writeDetailCSVOneWay(std::ofstream* output_log_csv_file, const EagleyeStates& eagleye_state)
{
    *output_log_csv_file << "timestamp,imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z,imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z\
,rtklib_nav.tow,rtklib_nav.ecef_pos.x,rtklib_nav.ecef_pos.y,rtklib_nav.ecef_pos.z,rtklib_nav.ecef_vel.x,rtklib_nav.ecef_vel.y,rtklib_nav.ecef_vel.z,rtklib_nav.status.status.status,rtklib_nav.status.status.service,rtklib_nav.status.latitude,rtklib_nav.status.longitude,rtklib_nav.status.altitude\
,velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.linear.z,velocity.twist.angular.x,velocity.twist.angular.y,velocity.twist.angular.z\
,velocity_scale_factor.scale_factor,velocity_scale_factor.correction_velocity.linear.x,velocity_scale_factor.correction_velocity.linear.y,velocity_scale_factor.correction_velocity.linear.z,velocity_scale_factor.correction_velocity.angular.x,velocity_scale_factor.correction_velocity.angular.y,velocity_scale_factor.correction_velocity.angular.z,velocity_scale_factor.status.enabled_status,velocity_scale_factor.status.estimate_status\
,distance.distance,distance.status.enabled_status,distance.status.estimate_status\
,heading_1st.heading_angle,heading_1st.status.enabled_status,heading_1st.status.estimate_status\
,heading_interpolate_1st.heading_angle,heading_interpolate_1st.status.enabled_status,heading_interpolate_1st.status.estimate_status\
,heading_2nd.heading_angle,heading_2nd.status.enabled_status,heading_2nd.status.estimate_status\
,heading_interpolate_2nd.heading_angle,heading_interpolate_2nd.status.enabled_status,heading_interpolate_2nd.status.estimate_status\
,heading_3rd.heading_angle,heading_3rd.status.enabled_status,heading_3rd.status.estimate_status\
,heading_interpolate_3rd.heading_angle,heading_interpolate_3rd.status.enabled_status,heading_interpolate_3rd.status.estimate_status\
,yawrate_offset_stop.yawrate_offset,yawrate_offset_stop.status.enabled_status,yawrate_offset_stop.status.estimate_status\
,yawrate_offset_1st.yawrate_offset,yawrate_offset_1st.status.enabled_status,yawrate_offset_1st.status.estimate_status\
,yawrate_offset_2nd.yawrate_offset,yawrate_offset_2nd.status.enabled_status,yawrate_offset_2nd.status.estimate_status\
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
    // ,angular_velocity_offset_stop.rollrate_offset,angular_velocity_offset_stop.pitchrate_offset,angular_velocity_offset_stop.yawrate_offset,angular_velocity_offset_stop.status.enabled_status,angular_velocity_offset_stop.status.estimate_status
    for(int i=0; i<data_length_; i++)
    {
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << imu_[i].header.stamp.toNSec() << ","; //timestamp
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].angular_velocity.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << imu_[i].linear_acceleration.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].tow << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].ecef_vel.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(rtklib_nav_[i].status.status.status) << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << rtklib_nav_[i].status.status.service << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.latitude << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.longitude << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << rtklib_nav_[i].status.altitude << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.linear.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.angular.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.angular.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << velocity_[i].twist.angular.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].scale_factor << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.linear.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.angular.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.angular.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.velocity_scale_factor[i].correction_velocity.angular.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.velocity_scale_factor[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.velocity_scale_factor[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.distance[i].distance << ",";
      *output_log_csv_file << (eagleye_state_forward_.distance[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.distance[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_1st[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_1st[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_2nd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_2nd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_3rd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_3rd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_3rd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.heading_interpolate_3rd[i].heading_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_3rd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.heading_interpolate_3rd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.yawrate_offset_stop[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_stop[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_stop[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.yawrate_offset_1st[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_1st[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_1st[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.yawrate_offset_2nd[i].yawrate_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_2nd[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.yawrate_offset_2nd[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.slip_angle[i].coefficient << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.slip_angle[i].slip_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.slip_angle[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.slip_angle[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_vel[i].vector.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_vel[i].vector.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_vel[i].vector.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].enu_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].ecef_base_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].ecef_base_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos[i].ecef_base_pos.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].enu_pos.z << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].ecef_base_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].ecef_base_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_absolute_pos_interpolate[i].ecef_base_pos.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_absolute_pos_interpolate[i].status.estimate_status ? "1" : "0") << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.angular_velocity_offset_stop.rollrate_offset << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.angular_velocity_offset_stop.pitchrate_offset << ",";
      // *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.angular_velocity_offset_stop.yawrate_offset << ",";
      // *output_log_csv_file << (eagleye_state_forward_.angular_velocity_offset_stop.status.enabled_status ? "1" : "0") << ",";
      // *output_log_csv_file << (eagleye_state_forward_.angular_velocity_offset_stop.status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.height[i].height << ",";
      *output_log_csv_file << (eagleye_state_forward_.height[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.height[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.pitching[i].pitching_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.pitching[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.pitching[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.acc_x_offset[i].acc_x_offset << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_offset[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_offset[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.acc_x_scale_factor[i].acc_x_scale_factor << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_scale_factor[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.acc_x_scale_factor[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.rolling[i].rolling_angle << ",";
      *output_log_csv_file << (eagleye_state_forward_.rolling[i].status.enabled_status ? "1" : "0") << ",";
      *output_log_csv_file << (eagleye_state_forward_.rolling[i].status.estimate_status ? "1" : "0") << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << gga_[i].header.stamp.toNSec() << ","; //timestamp
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lat << ","; //gga_llh.latitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].lon << ","; //gga_llh.longitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << gga_[i].alt +  gga_[i].undulation << ","; //gga_llh.altitude
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << int(gga_[i].gps_qual) << ","; //gga_llh.gps_qual
      if(getUseCombination())
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lat_[i] << ","; //eagleye_pp_llh.latitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_lon_[i] << ","; //eagleye_pp_llh.longitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << llh_smoothing_trajectory_hei_[i] << ","; //eagleye_pp_llh.altitude
      }
      else
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.latitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.longitude
        *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.altitude     
      }
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[0]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[1]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[2]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[3]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[4]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[5]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[6]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[7]
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << 0 << ","; //eagleye_pp_llh.orientation_covariance[8]
      if(getUseCombination())
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << smoothing_trajectory_status_[i] << ","; //eagleye_pp_llh.status
      }
      else
      {
        *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << 0 << ","; //eagleye_pp_llh.status    
      }
      *output_log_csv_file << std::setprecision(std::numeric_limits<int>::max_digits10) << eagleye_state_forward_.flag_reliability_buffer[i] << ","; //eagleye_pp_llh.status
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_relative_pos[i].enu_pos.x << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_relative_pos[i].enu_pos.y << ",";
      *output_log_csv_file << std::setprecision(std::numeric_limits<double>::max_digits10) << eagleye_state_forward_.enu_relative_pos[i].enu_pos.z << ",";
      *output_log_csv_file << (eagleye_state_forward_.enu_relative_pos[i].status.enabled_status ? "1" : "0");
      *output_log_csv_file << "\n";
    }
}

/************************************************************
writeDetailCSV
Function to output csv(Detailed version)
************************************************************/
void eagleye_pp::writeDetailCSV(void)
{

  if (!boost::filesystem::is_directory(outputpath_))
  {
    boost::filesystem::create_directory(outputpath_);
    std::cout << "Directory created: " << outputpath_ << std::endl;
  }

  if(output_log_)
  {
    std::ofstream output_log_csv_file(outputpath_ + "eagleye_log.csv", std::ios::out);
    std::cout << "Output file = " << outputpath_ << "eagleye_log.csv" << std::endl;
    writeDetailCSVOneWay(&output_log_csv_file, eagleye_state_forward_);

    if(getUseBackward())
    {
      std::ofstream output_log_back_csv_file(outputpath_ + "eagleye_log_back.csv", std::ios::out);
      std::cout << "Output file = " << outputpath_ << "eagleye_log_back.csv" << std::endl;
      writeDetailCSVOneWay(&output_log_back_csv_file, eagleye_state_backward_);
    }
  }
}
// end
