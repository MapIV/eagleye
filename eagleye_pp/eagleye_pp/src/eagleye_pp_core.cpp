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
  rtk_deadreckoning_parameter_ = {};
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

 std::string name = "eagleye";
  std::string logo_link_url = "https://github.com/MapIV/eagleye/blob/main-ros1/docs/logo.png";
  kml_generator_ = new KmlGenerator(outputpath_ + "ealgeye.kml", name, logo_link_url);
  kml_generator_line_ = new KmlGenerator(outputpath_ + "ealgeye_line.kml", name, logo_link_url);
}


/********************
setParam
*********************/
void eagleye_pp::setParam(std::string arg_config_file, std::string *arg_twist_topic, std::string *arg_imu_topic, std::string *arg_rtklib_nav_topic,
  std::string *arg_nmea_sentence_topic)
{
  YAML::Node conf = YAML::LoadFile(arg_config_file);
  config_file_ = arg_config_file;
  try
  {

    // Eagleye_pp params
    *arg_twist_topic = conf["twist_topic"].as<std::string>();
    *arg_imu_topic = conf["imu_topic"].as<std::string>();
    *arg_rtklib_nav_topic = conf["rtklib_nav_topic"].as<std::string>();
    *arg_nmea_sentence_topic = conf["nmea_sentence_topic"].as<std::string>();

    // output_log = conf["output_log"].as<bool>();
    output_log_ = true;
    // timestamp_sort = conf["timestamp_sort"].as<bool>();
    timestamp_sort_ = true;
    convert_height_num_ = conf["convert_height_num"].as<int>();
    interval_line_ = conf["interval_line"].as<double>();
    output_kml_eagleye_forward_plot_ = conf["output_kml_eagleye_forward_plot"].as<bool>();
    output_kml_eagleye_backward_plot_ = conf["output_kml_eagleye_backward_plot"].as<bool>();
    output_kml_eagleye_pp_plot_ = conf["output_kml_eagleye_pp_plot"].as<bool>();
    output_kml_rtklib_plot_ = conf["output_kml_rtklib_plot"].as<bool>();
    output_kml_gnss_plot_ = conf["output_kml_gnss_plot"].as<bool>();
    output_kml_eagleye_forward_line_ = conf["output_kml_eagleye_forward_line"].as<bool>();
    output_kml_eagleye_backward_line_ = conf["output_kml_eagleye_backward_line"].as<bool>();
    output_kml_eagleye_pp_line_ = conf["output_kml_eagleye_pp_line"].as<bool>();

    tf2::Quaternion tf2_quat;
    double x = conf["imu"]["base_link2imu"]["x"].as<double>();
    double y = conf["imu"]["base_link2imu"]["y"].as<double>();
    double z = conf["imu"]["base_link2imu"]["z"].as<double>();
    double roll = conf["imu"]["base_link2imu"]["roll"].as<double>();
    double pitch = conf["imu"]["base_link2imu"]["pitch"].as<double>();
    double yaw = conf["imu"]["base_link2imu"]["yaw"].as<double>();
    tf2_quat.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat = tf2::toMsg(tf2_quat);

    base_link2imu_.transform.translation.x = x;
    base_link2imu_.transform.translation.y = y;
    base_link2imu_.transform.translation.z = z;
    base_link2imu_.transform.rotation = geometry_quat;

    tf2::Quaternion tf2_quat_gnss;
    double gnss_roll = conf["gnss"]["base_link2gnss"]["roll"].as<double>();
    double gnss_pitch = conf["gnss"]["base_link2gnss"]["pitch"].as<double>();
    double gnss_yaw = conf["gnss"]["base_link2gnss"]["yaw"].as<double>();
    tf2_quat_gnss.setRPY(gnss_roll, gnss_pitch, gnss_yaw);
    geometry_msgs::Quaternion geometry_quat_gnss = tf2::toMsg(tf2_quat_gnss);
  
    // eagleye_rt params

    use_gnss_mode_ = conf["gnss"]["use_gnss_mode"].as<std::string>();
    use_nmea_downsample_ = conf["gnss"]["use_nmea_downsample"].as<bool>();
    nmea_downsample_freq_ = conf["gnss"]["nmea_downsample_freq"].as<double>();

    use_canless_mode_ = conf["use_canless_mode"].as<bool>();

    heading_interpolate_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    heading_interpolate_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    heading_interpolate_parameter_.sync_search_period = conf["heading_interpolate"]["sync_search_period"].as<double>();
    heading_interpolate_parameter_.proc_noise = conf["heading_interpolate"]["proc_noise"].as<double>();

    heading_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    heading_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    heading_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    heading_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    heading_parameter_.estimated_minimum_interval = conf["heading"]["estimated_minimum_interval"].as<double>();
    heading_parameter_.estimated_maximum_interval = conf["heading"]["estimated_maximum_interval"].as<double>();
    heading_parameter_.gnss_receiving_threshold = conf["heading"]["gnss_receiving_threshold"].as<double>();
    heading_parameter_.outlier_threshold = conf["heading"]["outlier_threshold"].as<double>();
    heading_parameter_.outlier_ratio_threshold = conf["heading"]["outlier_ratio_threshold"].as<double>();
    heading_parameter_.curve_judgment_threshold = conf["heading"]["curve_judgment_threshold"].as<double>();
    heading_parameter_.init_STD = conf["heading"]["init_STD"].as<double>();

    position_interpolate_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    position_interpolate_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    position_interpolate_parameter_.sync_search_period = conf["position_interpolate"]["sync_search_period"].as<double>();

    position_parameter_.ecef_base_pos_x = conf["ecef_base_pos"]["x"].as<double>();
    position_parameter_.ecef_base_pos_y = conf["ecef_base_pos"]["y"].as<double>();
    position_parameter_.ecef_base_pos_z = conf["ecef_base_pos"]["z"].as<double>();
    // position_parameter_.tf_gnss_parent_frame = conf["tf_gnss_frame"]["parent"].as<std::string>();
    // position_parameter_.tf_gnss_child_frame = conf["tf_gnss_frame"]["child"].as<std::string>();
    position_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    position_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    position_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    position_parameter_.estimated_interval = conf["position"]["estimated_interval"].as<double>();
    position_parameter_.update_distance = conf["position"]["update_distance"].as<double>();
    position_parameter_.outlier_threshold = conf["position"]["outlier_threshold"].as<double>();
    position_parameter_.gnss_receiving_threshold = conf["heading"]["gnss_receiving_threshold"].as<double>();
    position_parameter_.outlier_ratio_threshold = conf["position"]["outlier_ratio_threshold"].as<double>();

    rtk_deadreckoning_parameter_.ecef_base_pos_x = conf["ecef_base_pos"]["x"].as<double>();
    rtk_deadreckoning_parameter_.ecef_base_pos_y = conf["ecef_base_pos"]["y"].as<double>();
    rtk_deadreckoning_parameter_.ecef_base_pos_z = conf["ecef_base_pos"]["z"].as<double>();
    rtk_deadreckoning_parameter_.use_ecef_base_position = conf["ecef_base_pos"]["use_ecef_base_position"].as<bool>();
    // rtk_deadreckoning_parameter_.tf_gnss_parent_frame = conf["tf_gnss_frame"]["parent"].as<std::string>();
    // rtk_deadreckoning_parameter_.tf_gnss_child_frame = conf["tf_gnss_frame"]["child"].as<std::string>();
    rtk_deadreckoning_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    rtk_deadreckoning_parameter_.rtk_fix_STD = conf["rtk_deadreckoning"]["rtk_fix_STD"].as<double>();
    rtk_deadreckoning_parameter_.proc_noise = conf["rtk_deadreckoning"]["proc_noise"].as<double>();

    slip_angle_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    slip_angle_parameter_.manual_coefficient = conf["slip_angle"]["manual_coefficient"].as<double>();

    smoothing_parameter_.ecef_base_pos_x = conf["ecef_base_pos"]["x"].as<double>();
    smoothing_parameter_.ecef_base_pos_y = conf["ecef_base_pos"]["y"].as<double>();
    smoothing_parameter_.ecef_base_pos_z = conf["ecef_base_pos"]["z"].as<double>();
    smoothing_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    smoothing_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    smoothing_parameter_.moving_average_time = conf["smoothing"]["moving_average_time"].as<double>();
    smoothing_parameter_.moving_ratio_threshold = conf["smoothing"]["moving_ratio_threshold"].as<double>();

    trajectory_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    trajectory_parameter_.curve_judgment_threshold = conf["trajectory"]["curve_judgment_threshold"].as<double>();
    trajectory_parameter_.sensor_noise_velocity = conf["trajectory"]["sensor_noise_velocity"].as<double>();
    trajectory_parameter_.sensor_scale_noise_velocity = conf["trajectory"]["sensor_scale_noise_velocity"].as<double>();
    trajectory_parameter_.sensor_noise_yawrate = conf["trajectory"]["sensor_noise_yawrate"].as<double>();
    trajectory_parameter_.sensor_bias_noise_yawrate = conf["trajectory"]["sensor_bias_noise_yawrate"].as<double>();

    velocity_scale_factor_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    velocity_scale_factor_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    velocity_scale_factor_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    velocity_scale_factor_parameter_.estimated_minimum_interval = conf["velocity_scale_factor"]["estimated_minimum_interval"].as<double>();
    velocity_scale_factor_parameter_.estimated_maximum_interval = conf["velocity_scale_factor"]["estimated_maximum_interval"].as<double>();
    velocity_scale_factor_parameter_.gnss_receiving_threshold = conf["velocity_scale_factor"]["gnss_receiving_threshold"].as<double>();

    yawrate_offset_1st_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    yawrate_offset_1st_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    yawrate_offset_1st_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    yawrate_offset_1st_parameter_.estimated_minimum_interval = conf["yawrate_offset"]["estimated_minimum_interval"].as<double>();
    yawrate_offset_1st_parameter_.estimated_maximum_interval = conf["yawrate_offset"]["1st"]["estimated_maximum_interval"].as<double>();
    yawrate_offset_1st_parameter_.gnss_receiving_threshold = conf["yawrate_offset"]["gnss_receiving_threshold"].as<double>();
    yawrate_offset_1st_parameter_.outlier_threshold = conf["yawrate_offset_stop"]["outlier_threshold"].as<double>();

    yawrate_offset_2nd_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    yawrate_offset_2nd_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    yawrate_offset_2nd_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    yawrate_offset_2nd_parameter_.estimated_minimum_interval = conf["yawrate_offset"]["estimated_minimum_interval"].as<double>();
    yawrate_offset_2nd_parameter_.estimated_maximum_interval = conf["yawrate_offset"]["2nd"]["estimated_maximum_interval"].as<double>();
    yawrate_offset_2nd_parameter_.gnss_receiving_threshold = conf["yawrate_offset"]["gnss_receiving_threshold"].as<double>();
    yawrate_offset_2nd_parameter_.outlier_threshold = conf["yawrate_offset_stop"]["outlier_threshold"].as<double>();

    yawrate_offset_stop_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    yawrate_offset_stop_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    yawrate_offset_stop_parameter_.estimated_interval = conf["yawrate_offset_stop"]["estimated_interval"].as<double>();
    yawrate_offset_stop_parameter_.outlier_threshold = conf["yawrate_offset_stop"]["outlier_threshold"].as<double>();

    height_parameter_.imu_rate = conf["common"]["imu_rate"].as<double>();
    height_parameter_.gnss_rate = conf["common"]["gnss_rate"].as<double>();
    height_parameter_.moving_judgment_threshold = conf["common"]["moving_judgment_threshold"].as<double>();
    height_parameter_.estimated_minimum_interval = conf["height"]["estimated_minimum_interval"].as<double>();
    height_parameter_.estimated_maximum_interval = conf["height"]["estimated_maximum_interval"].as<double>();
    height_parameter_.update_distance = conf["height"]["update_distance"].as<double>();
    height_parameter_.gnss_receiving_threshold = conf["height"]["gnss_receiving_threshold"].as<double>();
    height_parameter_.outlier_threshold = conf["height"]["outlier_threshold"].as<double>();
    height_parameter_.outlier_ratio_threshold = conf["height"]["outlier_ratio_threshold"].as<double>();
    height_parameter_.moving_average_time = conf["height"]["moving_average_time"].as<double>();

    rolling_parameter_.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    rolling_parameter_.filter_process_noise = conf["rolling"]["filter_process_noise"].as<double>();
    rolling_parameter_.filter_observation_noise = conf["rolling"]["filter_observation_noise"].as<double>();
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

bool eagleye_pp::getUseCanlessMode(void)
{
 return use_canless_mode_;
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

sensor_msgs::Imu eagleye_pp::transformIMU(sensor_msgs::Imu imu_msg)
{
  sensor_msgs::Imu transformed_imu_msg;
  transformed_imu_msg.header = imu_msg.header;

  transformed_imu_msg.header = imu_msg.header;
  geometry_msgs::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
  geometry_msgs::Quaternion  transformed_quaternion;

  angular_velocity.header = imu_msg.header;
  angular_velocity.vector = imu_msg.angular_velocity;
  linear_acceleration.header = imu_msg.header;
  linear_acceleration.vector = imu_msg.linear_acceleration;

  tf2::doTransform(angular_velocity, transformed_angular_velocity, base_link2imu_);
  tf2::doTransform(linear_acceleration, transformed_linear_acceleration, base_link2imu_);
  tf2::doTransform(imu_msg.orientation, transformed_quaternion, base_link2imu_);

  transformed_imu_msg.angular_velocity = transformed_angular_velocity.vector;
  transformed_imu_msg.linear_acceleration = transformed_linear_acceleration.vector;
  transformed_imu_msg.orientation = transformed_quaternion;
  return transformed_imu_msg;
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
        sensor_msgs::Imu transformed_imu_msg = transformIMU(imu_msg);
        imu_.push_back(transformed_imu_msg);
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
  struct RtkDeadreckoningStatus rtk_deadreckoning_status{};

  geometry_msgs::TwistStamped _correction_velocity;
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
  geometry_msgs::TwistWithCovarianceStamped _eagleye_twist_with_covariance;
  eagleye_msgs::StatusStamped _velocity_status;

  VelocityEstimator velocity_estimator;
  velocity_estimator.setParam(config_file_);


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

    if(use_canless_mode_)
    {
      velocity_estimator.VelocityEstimate(imu_[i], rtklib_nav_[i], gga_[i], &_correction_velocity);
      _velocity_status.header = imu_[i].header;
      _velocity_status.status = velocity_estimator.getStatus();
        // std::cout << "_velocity_status: " << _velocity_status << std::flush;
    }
    else
    {
      _velocity_scale_factor.header = imu_[i].header;
      _correction_velocity.header = imu_[i].header;
      if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
        velocity_scale_factor_estimate(rtklib_nav_[i], velocity_[i], velocity_scale_factor_parameter_, &velocity_scale_factor_status, &_correction_velocity, &_velocity_scale_factor);
      else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
        velocity_scale_factor_estimate(rmc_[i], velocity_[i], velocity_scale_factor_parameter_, &velocity_scale_factor_status, &_correction_velocity, &_velocity_scale_factor);

      _velocity_status.header = _velocity_scale_factor.header;
      _velocity_status.status = _velocity_scale_factor.status;
    }

    if(!use_canless_mode_ || _velocity_status.status.enabled_status)
    {
    _slip_angle.header = imu_[i].header;
    slip_angle_estimate(imu_[i], _correction_velocity, _velocity_status, _yawrate_offset_stop, _yawrate_offset_2nd, slip_angle_parameter_, &_slip_angle);

    _gnss_smooth_pos_enu.header = imu_[i].header;
    smoothing_estimate(rtklib_nav_[i], _correction_velocity, smoothing_parameter_, &smoothing_status, &_gnss_smooth_pos_enu);

    _yawrate_offset_stop.header = imu_[i].header;
    yawrate_offset_stop_estimate(_correction_velocity, imu_[i], yawrate_offset_stop_parameter_, &yawrate_offset_stop_status, &_yawrate_offset_stop);

    _heading_1st.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      heading_estimate(rtklib_nav_[i], imu_[i],_correction_velocity, _yawrate_offset_stop, _yawrate_offset_stop, _slip_angle, _heading_interpolate_1st,
        heading_parameter_, &heading_1st_status, &_heading_1st);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      heading_estimate(rmc_[i], imu_[i],_correction_velocity, _yawrate_offset_stop, _yawrate_offset_stop, _slip_angle, _heading_interpolate_1st,
        heading_parameter_, &heading_1st_status, &_heading_1st);

    _heading_interpolate_1st.header = imu_[i].header;
    heading_interpolate_estimate(imu_[i], _correction_velocity, _yawrate_offset_stop, _yawrate_offset_stop, _heading_1st, _slip_angle,
      heading_interpolate_parameter_, &heading_interpolate_1st_status, &_heading_interpolate_1st);

    _yawrate_offset_1st.header = imu_[i].header;
    yawrate_offset_estimate(_correction_velocity, _yawrate_offset_stop, _heading_interpolate_1st, imu_[i], yawrate_offset_1st_parameter_,
      &yawrate_offset_1st_status, &_yawrate_offset_1st);

    _heading_2nd.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      heading_estimate(rtklib_nav_[i], imu_[i], _correction_velocity, _yawrate_offset_stop, _yawrate_offset_1st, _slip_angle,
        _heading_interpolate_2nd, heading_parameter_, &heading_2nd_status, &_heading_2nd);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      heading_estimate(rmc_[i], imu_[i], _correction_velocity, _yawrate_offset_stop, _yawrate_offset_1st, _slip_angle, _heading_interpolate_2nd,
        heading_parameter_, &heading_2nd_status, &_heading_2nd);

    _heading_interpolate_2nd.header = imu_[i].header;
    heading_interpolate_estimate(imu_[i], _correction_velocity, _yawrate_offset_stop, _yawrate_offset_1st, _heading_2nd, _slip_angle,
      heading_interpolate_parameter_, &heading_interpolate_2nd_status, &_heading_interpolate_2nd);

    _yawrate_offset_2nd.header = imu_[i].header;
    yawrate_offset_estimate(_correction_velocity, _yawrate_offset_stop, _heading_interpolate_2nd, imu_[i], yawrate_offset_2nd_parameter_,
      &yawrate_offset_2nd_status, &_yawrate_offset_2nd);

    _heading_3rd.header = imu_[i].header;
    if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
      heading_estimate(rtklib_nav_[i], imu_[i], _correction_velocity, _yawrate_offset_2nd, _yawrate_offset_stop, _slip_angle, _heading_interpolate_3rd,
        heading_parameter_, &heading_3rd_status, &_heading_3rd);
    else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
      heading_estimate(rmc_[i], imu_[i], _correction_velocity, _yawrate_offset_2nd, _yawrate_offset_stop, _slip_angle, _heading_interpolate_3rd,
        heading_parameter_, &heading_3rd_status, &_heading_3rd);

    _heading_interpolate_3rd.header = imu_[i].header;
    heading_interpolate_estimate(imu_[i], _correction_velocity, _yawrate_offset_2nd, _yawrate_offset_stop, _heading_3rd, _slip_angle,
      heading_interpolate_parameter_, &heading_interpolate_3rd_status, &_heading_interpolate_3rd);

    _distance.header = imu_[i].header;
    if(_distance.header.stamp.toSec() != 0)
    {
      distance_estimate(_correction_velocity, &distance_status, &_distance);
    }

    _height.header = imu_[i].header;
    _pitching.header = imu_[i].header;
    _acc_x_offset.header = imu_[i].header;
    _acc_x_scale_factor.header = imu_[i].header;
    pitching_estimate(imu_[i], gga_[i], _correction_velocity, _distance, height_parameter_, &height_status, &_height, &_pitching,
      &_acc_x_offset, &_acc_x_scale_factor);

    rolling_estimate(imu_[i], _correction_velocity, _yawrate_offset_stop, _yawrate_offset_2nd, rolling_parameter_, &rolling_status, &_rolling);

    _enu_vel.header = imu_[i].header;
    _enu_relative_pos.header = imu_[i].header;
    _eagleye_twist.header = imu_[i].header;
    trajectory_estimate(imu_[i], _correction_velocity, _velocity_status, _heading_interpolate_3rd, _yawrate_offset_stop, _yawrate_offset_2nd, trajectory_parameter_, 
      &trajectory_status, &_enu_vel, &_enu_relative_pos, &_eagleye_twist, &_eagleye_twist_with_covariance);

    _enu_absolute_pos.header = imu_[i].header;
    _enu_absolute_pos_interpolate.header = imu_[i].header;
    _eagleye_fix.header = imu_[i].header;
    if(use_canless_mode_)
    {
      if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB") 
        rtk_deadreckoning_estimate(rtklib_nav_[i], _enu_vel, gga_[i], _heading_interpolate_3rd,
          rtk_deadreckoning_parameter_, &rtk_deadreckoning_status, &_enu_absolute_pos_interpolate, &_eagleye_fix);
      else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA") 
        rtk_deadreckoning_estimate(_enu_vel, gga_[i], _heading_interpolate_3rd,
          rtk_deadreckoning_parameter_, &rtk_deadreckoning_status, &_enu_absolute_pos_interpolate, &_eagleye_fix);      
    }
    else
    {
      if (use_gnss_mode_ == "rtklib" || use_gnss_mode_ == "RTKLIB")
        position_estimate(rtklib_nav_[i], _correction_velocity, _velocity_status, _distance, _heading_interpolate_3rd, _enu_vel, position_parameter_, &position_status,
          &_enu_absolute_pos);
      else if (use_gnss_mode_ == "nmea" || use_gnss_mode_ == "NMEA")
        position_estimate(gga_[i], _correction_velocity, _velocity_status, _distance, _heading_interpolate_3rd, _enu_vel, position_parameter_, &position_status, &_enu_absolute_pos);

      position_interpolate_estimate(_enu_absolute_pos, _enu_vel, _gnss_smooth_pos_enu, _height, position_interpolate_parameter_, &position_interpolate_status,
        &_enu_absolute_pos_interpolate, &_eagleye_fix);
    }
    }

    if(arg_forward_flag)
    {
      eagleye_state_forward_.correction_velocity.push_back(_correction_velocity);
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
    	eagleye_state_forward_.eagleye_twist_with_covariance.push_back(_eagleye_twist_with_covariance);
    	eagleye_state_forward_.flag_reliability_buffer.push_back(height_status.flag_reliability);
    }
    else
    {
      eagleye_state_backward_.correction_velocity.push_back(_correction_velocity);
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
    	eagleye_state_backward_.eagleye_twist_with_covariance.push_back(_eagleye_twist_with_covariance);
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
    std::reverse(eagleye_state_backward_.correction_velocity.begin(), eagleye_state_backward_.correction_velocity.end());
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
    _distance[i] = _distance[i-1] + eagleye_state_forward_.correction_velocity[i].twist.linear.x * (arg_GPSTime[i] - arg_GPSTime[i-1]);
  }
  for(int i = 0; i < datanum; i++){
    int index_Dist = -1;
    for(int k = 0; k < datanum; k++){
      if(_distance[k] > _distance[i] - ESTDIST){
        index_Dist = k;
        break;
      }
    }

    if (_distance[i] > ESTDIST && flag_GNSS_[i] == 1 && eagleye_state_forward_.correction_velocity[i].twist.linear.x > TH_VEL_EST &&
      index_Dist > index_Raw[0]){
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
	if(eagleye_state_forward_.correction_velocity[i_start + j].twist.linear.x > TH_VEL_EST ){
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
    } // if (_distance[i] > ESTDIST && flag_GNSS_[i] == 1 && correction_velocity[i].twist.linear.x > TH_VEL_EST && index_Dist > index_Raw[0])
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
    _distance[i] = _distance[i-1] + eagleye_state_forward_.correction_velocity[i].twist.linear.x * (arg_GPSTime[i] - arg_GPSTime[i-1]);
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
    slip[i] = eagleye_state_forward_.correction_velocity[i].twist.linear.x * Yawrate_Est[i] * slip_angle_parameter_.manual_coefficient;
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
      double correction_velocity_x = eagleye_state_forward_.correction_velocity[i].twist.linear.x;
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
          pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
            sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
          pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
            cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
        }else if( Tramodelswitch == 1){    
          if(abs(Yawrate_Est[i]) > TH_Yaw){
             pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
              ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+
              cos(Heading_IMU_slip[i-1]));
             pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
              ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+(Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-
              sin(Heading_IMU_slip[i-1]));
          }else{
            pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
              sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
              cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
          }
        } // else if( Tramodelswitch == 1)
      }else if( i > 0){
        if(Tramodelswitch == 0){  
          if(switch_tmp == 0){
            pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
              sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
              cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
          }else if(switch_tmp == 1){
            if(flag_DRs[i-1] == 1){
              pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i-1 + datanum * 0] +
                sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              pUsrPos_FixSlip[i + datanum * 1]= UsrPos_TaGRTK_enu[i-1 + datanum * 1] +
                cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            }else{
              pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
                sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
                cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[I] - arg_GPSTime[i-1]);
            }
          } // else if(switch_tmp == 1)
        }else if(Tramodelswitch == 1){     
          if(switch_tmp == 0){
            if(abs(Yawrate_Est[i]) > TH_Yaw){
              pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
                ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+
                (Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
              pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
                ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+
                (Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
            }else{
              pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
                sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
                cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
            }
          }else if(switch_tmp == 1){
            if (flag_DRs[i-1] == 1){
              if (abs(Yawrate_Est[i]) > TH_Yaw){
                pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i-1 + datanum * 0] +
                  ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+
                  (Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
                pUsrPos_FixSlip[i + datanum * 1]= UsrPos_TaGRTK_enu[i-1 + datanum * 1] +
                  ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+
                  (Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
              }else{
                pUsrPos_FixSlip[i + datanum * 0] = UsrPos_TaGRTK_enu[i-1 + datanum * 0] +
                  sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
                pUsrPos_FixSlip[i + datanum * 1]= UsrPos_TaGRTK_enu[i-1 + datanum * 1] +
                  cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
              }
	    }else{ 
              if(abs(Yawrate_Est[i]) > TH_Yaw){
                pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
                  ((correction_velocity_x)/Yawrate_Est[i])*(-cos(Heading_IMU_slip[i-1]+
                  (Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))+cos(Heading_IMU_slip[i-1]));
                pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
                  ((correction_velocity_x)/Yawrate_Est[i])*(sin(Heading_IMU_slip[i-1]+
                  (Yawrate_Est[i]*(arg_GPSTime[i] - arg_GPSTime[i-1])))-sin(Heading_IMU_slip[i-1]));
              }else{
                pUsrPos_FixSlip[i + datanum * 0] = pUsrPos_FixSlip[i-1 + datanum * 0] +
                  sin(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
                pUsrPos_FixSlip[i + datanum * 1]= pUsrPos_FixSlip[i-1 + datanum * 1] +
                  cos(Heading_IMU_slip[i])*correction_velocity_x*(arg_GPSTime[i] - arg_GPSTime[i-1]);
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
    
    eagleye_msgs::StatusStamped velocity_enable_status;
    velocity_enable_status.header = eagleye_state_forward_.velocity_scale_factor[i].header;
    velocity_enable_status.status = eagleye_state_forward_.velocity_scale_factor[i].status;

    trajectory_estimate(imu_[i], eagleye_state_forward_.correction_velocity[i], velocity_enable_status, eagleye_state_forward_.heading_interpolate_3rd[i], 
      eagleye_state_forward_.yawrate_offset_stop[i], eagleye_state_forward_.yawrate_offset_2nd[i],
      trajectory_parameter_, &trajectory_status, &eagleye_state_forward_.enu_vel[i], &eagleye_state_forward_.enu_relative_pos[i],
      &eagleye_state_forward_.eagleye_twist[i], &eagleye_state_forward_.eagleye_twist_with_covariance[i]);
  }

  gga_length = std::distance(gga_.begin(), gga_.end());
  enu_relative_pos_length = std::distance(eagleye_state_forward_.enu_relative_pos.begin(), eagleye_state_forward_.enu_relative_pos.end());
  enu_absolute_pos_length = std::distance(eagleye_state_forward_.enu_absolute_pos_interpolate.begin(), eagleye_state_forward_.enu_absolute_pos_interpolate.end());
  velocity_scale_factor_length = std::distance(eagleye_state_forward_.velocity_scale_factor.begin(), eagleye_state_forward_.velocity_scale_factor.end());
  flag_reliability_buffer_length = std::distance(eagleye_state_forward_.flag_reliability_buffer.begin(), eagleye_state_forward_.flag_reliability_buffer.end());

  if(gga_length > 0 && gga_length == enu_relative_pos_length && gga_length == enu_absolute_pos_length && gga_length == velocity_scale_factor_length)
  {
    if (eagleye_state_forward_.enu_absolute_pos_interpolate[enu_absolute_pos_length-1].status.enabled_status == true)
    {
      tmp_ecef_base[0] = eagleye_state_forward_.enu_absolute_pos_interpolate[enu_absolute_pos_length-1].ecef_base_pos.x;
      tmp_ecef_base[1] = eagleye_state_forward_.enu_absolute_pos_interpolate[enu_absolute_pos_length-1].ecef_base_pos.y;
      tmp_ecef_base[2] = eagleye_state_forward_.enu_absolute_pos_interpolate[enu_absolute_pos_length-1].ecef_base_pos.z;

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
            tmp_vel.push_back(eagleye_state_forward_.correction_velocity[index_gga[i-1]+1 + j].twist.linear.x);
            all_vel.push_back(eagleye_state_forward_.correction_velocity[index_gga[i-1]+1 + j].twist.linear.x);
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
                (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.x + tmp_enu_r[0] -
                eagleye_state_forward_.enu_relative_pos[index_gga[i-1] + interval_count].enu_pos.x);
              diff_pos_north = (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.y + tmp_enu_f[1] -
                eagleye_state_forward_.enu_relative_pos[index_gga[i-1]].enu_pos.y) -
                (eagleye_state_forward_.enu_relative_pos[index_gga[i-1]+j].enu_pos.y + tmp_enu_r[1] -
                eagleye_state_forward_.enu_relative_pos[index_gga[i-1] + interval_count].enu_pos.y);
              diff_pos_2d = sqrt( pow((diff_pos_east), 2.0) + pow((diff_pos_north), 2.0));
            }

            tmp_vel_begin = eagleye_state_forward_.correction_velocity[index_gga[i-1]+1].twist.linear.x;
            tmp_vle_end = eagleye_state_forward_.correction_velocity[index_gga[i-1] + j-1].twist.linear.x;
            all_vel_begin  = eagleye_state_forward_.correction_velocity[index_gga[i-1]+1].twist.linear.x;
            all_vel_end = eagleye_state_forward_.correction_velocity[index_gga[i]].twist.linear.x;

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
      convert_height.setLLH(eagleye_state_forward_.eagleye_fix[i].latitude, eagleye_state_forward_.eagleye_fix[i].longitude,
        eagleye_state_forward_.eagleye_fix[i].altitude);
      eagleye_state_forward_.eagleye_fix[i].altitude = convert_height.convert2altitude();
      convert_height.setLLH(eagleye_state_backward_.eagleye_fix[i].latitude, eagleye_state_backward_.eagleye_fix[i].longitude,
        eagleye_state_backward_.eagleye_fix[i].altitude);
      eagleye_state_backward_.eagleye_fix[i].altitude = convert_height.convert2altitude();
      convert_height.setLLH(llh_smoothing_trajectory_lat_[i], llh_smoothing_trajectory_lon_[i],
        llh_smoothing_trajectory_hei_[i]);
      llh_smoothing_trajectory_hei_[i] = convert_height.convert2altitude();
    }
  }
  else if(convert_height_num_ == 2)
  {
    for(int i = 0; i < data_length_; i++)
    {
      convert_height.setLLH(rtklib_nav_[i].status.latitude, rtklib_nav_[i].status.longitude, rtklib_nav_[i].status.altitude);
      rtklib_nav_[i].status.altitude = convert_height.convert2ellipsoid();
      convert_height.setLLH(eagleye_state_forward_.eagleye_fix[i].latitude, eagleye_state_forward_.eagleye_fix[i].longitude,
        eagleye_state_forward_.eagleye_fix[i].altitude);
      eagleye_state_forward_.eagleye_fix[i].altitude = convert_height.convert2ellipsoid();
      convert_height.setLLH(eagleye_state_backward_.eagleye_fix[i].latitude, eagleye_state_backward_.eagleye_fix[i].longitude,
        eagleye_state_backward_.eagleye_fix[i].altitude);
      eagleye_state_backward_.eagleye_fix[i].altitude = convert_height.convert2ellipsoid();
      convert_height.setLLH(llh_smoothing_trajectory_lat_[i], llh_smoothing_trajectory_lon_[i], llh_smoothing_trajectory_hei_[i]);
      llh_smoothing_trajectory_hei_[i] = convert_height.convert2ellipsoid();
    }
  }
}
// end
