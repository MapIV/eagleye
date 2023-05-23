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
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * velocity_estimator.cpp
 * Author MapIV Takanose
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#define STATUS_FIX 4
#define g 9.80665

//---PitchrateOffsetStopEstimator----------------------------------------------------------------------------------------------------------
VelocityEstimator::PitchrateOffsetStopEstimator::PitchrateOffsetStopEstimator()
{
  stop_count = 0;
  pitch_rate_offset = 0;
  pitch_rate_offset_status.enabled_status = false;
}

void VelocityEstimator::PitchrateOffsetStopEstimator::setParam(std::string yaml_file)
{
  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);
    param.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    param.estimated_interval = conf["/**"]["ros__parameters"]["velocity_estimator"]["pitch_rate_offset"]["estimated_interval"].as<double>();
    param.buffer_count_max = param.imu_rate * param.estimated_interval;
    // std::cout<< "imu_rate "<<param.imu_rate<<std::endl;
    // std::cout<< "estimated_interval "<<param.estimated_interval<<std::endl;  
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mPitchrateOffsetStopEstimator YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
}

bool VelocityEstimator::PitchrateOffsetStopEstimator::PitchrateOffsetStopEstimate(double pitch_rate, double stop_status)
{
  pitch_rate_offset_status.estimate_status = false;
  pitch_rate_buffer.push_back(pitch_rate);
  std::size_t pitch_rate_buffer_length = std::distance(pitch_rate_buffer.begin(), pitch_rate_buffer.end());

  ++stop_count;

  if(!stop_status)
  {
    stop_count = 0;
    return pitch_rate_offset_status.enabled_status;
  }

  if (pitch_rate_buffer_length < param.buffer_count_max) return pitch_rate_offset_status.enabled_status;
  pitch_rate_buffer.erase(pitch_rate_buffer.begin());

  if (stop_count > param.buffer_count_max)
  {
    double accumulation_pitch_rate = 0.0;
    for (int i = 0; i < param.buffer_count_max/2; i++)
    {
      accumulation_pitch_rate += pitch_rate_buffer[i];
    }

    pitch_rate_offset = -1 * accumulation_pitch_rate / (param.buffer_count_max/2);
    pitch_rate_offset_status.enabled_status = true;
    pitch_rate_offset_status.estimate_status = true;
  }

  return pitch_rate_offset_status.enabled_status;
}

//---PitchingEstimator---------------------------------------------------------------------------------------------------------------------
VelocityEstimator::PitchingEstimator::PitchingEstimator()
{
  pitching = 0;
  pitching_status.enabled_status = false;
}

void VelocityEstimator::PitchingEstimator::setParam(std::string yaml_file)
{
  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);
    param.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    param.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    param.estimated_interval = conf["/**"]["ros__parameters"]["velocity_estimator"]["pitching"]["estimated_interval"].as<double>();
    param.buffer_max = param.imu_rate * param.estimated_interval;
    param.outlier_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["pitching"]["outlier_threshold"].as<double>();
    param.slow_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["slow_judgment_threshold"].as<double>();
    param.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["pitching"]["gnss_receiving_threshold"].as<double>();
    param.estimated_gnss_coefficient = param.gnss_rate/param.imu_rate * param.gnss_receiving_threshold;
    param.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["pitching"]["outlier_ratio_threshold"].as<double>();
    param.estimated_coefficient = param.estimated_gnss_coefficient * param.outlier_ratio_threshold;

    // std::cout<< "imu_rate "<<param.imu_rate<<std::endl;
    // std::cout<< "gnss_rate "<<param.gnss_rate<<std::endl;
    // std::cout<< "estimated_interval "<<param.estimated_interval<<std::endl;
    // std::cout<< "buffer_max "<<param.buffer_max<<std::endl;
    // std::cout<< "outlier_threshold "<<param.outlier_threshold<<std::endl;
    // std::cout<< "slow_judgment_threshold "<<param.slow_judgment_threshold<<std::endl;
    // std::cout<< "gnss_receiving_threshold "<<param.gnss_receiving_threshold<<std::endl;
    // std::cout<< "estimated_gnss_coefficient "<<param.estimated_gnss_coefficient<<std::endl;
    // std::cout<< "outlier_ratio_threshold "<<param.outlier_ratio_threshold<<std::endl;
    // std::cout<< "estimated_coefficient "<<param.estimated_coefficient<<std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mPitchingEstimator YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
}

bool VelocityEstimator::PitchingEstimator::PitchingEstimate
(double imu_time_last, double doppler_velocity, double rtkfix_velocity,
 double pitch_rate, double pitch_rate_offset, double rtkfix_pitching,
 bool navsat_update_status, bool stop_status)
{
  bool use_gnss_status = false;

  if (doppler_velocity > param.slow_judgment_threshold ||
      rtkfix_velocity > param.slow_judgment_threshold)
  {
    use_gnss_status = true;
  }

  // data buffer generate
  time_buffer.push_back(imu_time_last);
  corrected_pitch_rate_buffer.push_back(pitch_rate + pitch_rate_offset);
  rtkfix_pitching_buffer.push_back(rtkfix_pitching);
  navsat_update_status_buffer.push_back(navsat_update_status);
  use_gnss_status_buffer.push_back(use_gnss_status);

  std::size_t time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());
  if (time_buffer_length <= param.buffer_max) return pitching_status.enabled_status;

  time_buffer.erase(time_buffer.begin());
  corrected_pitch_rate_buffer.erase(corrected_pitch_rate_buffer.begin());
  rtkfix_pitching_buffer.erase(rtkfix_pitching_buffer.begin());
  navsat_update_status_buffer.erase(navsat_update_status_buffer.begin());
  use_gnss_status_buffer.erase(use_gnss_status_buffer.begin());
  time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());

  // setup for estimation
  double accumulated_pitch_rate;
  std::vector<double> accumulated_pitch_rate_buffer(time_buffer_length);
  std::vector<int> gnss_index;
  std::size_t gnss_index_length;
  pitching_status.estimate_status = false;

  for (int i = 0; i < time_buffer_length; i++)
  {
    if (navsat_update_status_buffer[i] && use_gnss_status_buffer[i]) gnss_index.push_back(i); //TODO Velocity judgment
    if (i == 0) continue;
    accumulated_pitch_rate += corrected_pitch_rate_buffer[i] * (time_buffer[i]-time_buffer[i-1]);
    accumulated_pitch_rate_buffer[i] = accumulated_pitch_rate;
  }

  gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());

  if (gnss_index_length < time_buffer_length * param.estimated_gnss_coefficient) return pitching_status.enabled_status;

  // pitching estimation by the least-squares method
  std::vector<double> init_accumulated_pitch_rate_buffer;
  std::vector<double> residual_error;

  while (1)
  {
    gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());

    for (int i = 0; i < time_buffer_length; i++)
    {
      init_accumulated_pitch_rate_buffer.push_back(accumulated_pitch_rate_buffer[i] + (rtkfix_pitching_buffer[gnss_index[0]] - accumulated_pitch_rate_buffer[gnss_index[0]]));
    }

    for (int i = 0; i < gnss_index_length; i++)
    {
      residual_error.push_back(init_accumulated_pitch_rate_buffer[gnss_index[i]] - rtkfix_pitching_buffer[gnss_index[i]]);
    }

    double residual_error_mean = std::accumulate(residual_error.begin(), residual_error.end(), 0.0) / gnss_index_length;
    double init_pitching = rtkfix_pitching_buffer[gnss_index[0]] - residual_error_mean;

    init_accumulated_pitch_rate_buffer.clear();
    for (int i = 0; i < time_buffer_length; i++)
    {
      init_accumulated_pitch_rate_buffer.push_back(accumulated_pitch_rate_buffer[i] + (init_pitching - accumulated_pitch_rate_buffer[gnss_index[0]]));
    }

    residual_error.clear();
    for (int i = 0; i < gnss_index_length; i++)
    {
      residual_error.push_back(init_accumulated_pitch_rate_buffer[gnss_index[i]] - rtkfix_pitching_buffer[gnss_index[i]]);
    }

    std::vector<double>::iterator residual_error_max = std::max_element(residual_error.begin(), residual_error.end());
    int residual_error_max_index = std::distance(residual_error.begin(), residual_error_max);

    if (residual_error[residual_error_max_index] < param.outlier_threshold) break;
    gnss_index.erase(gnss_index.begin() + residual_error_max_index);
    gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());
    if (gnss_index_length < param.buffer_max  * param.estimated_coefficient) break;

  }

  pitching = init_accumulated_pitch_rate_buffer[time_buffer_length-1];
  pitching_status.enabled_status = true;
  pitching_status.estimate_status = true;

  return pitching_status.enabled_status;
}

//---AccelerationOffsetEstimator-----------------------------------------------------------------------------------------------------------
VelocityEstimator::AccelerationOffsetEstimator::AccelerationOffsetEstimator()
{
  acceleration_offset = 0;
  acceleration_offset_status.enabled_status = false;
}

void VelocityEstimator::AccelerationOffsetEstimator::setParam(std::string yaml_file)
{
  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);
    param.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    param.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    param.estimated_minimum_interval = conf["/**"]["ros__parameters"]["velocity_estimator"]["acceleration_offset"]["estimated_minimum_interval"].as<double>();
    param.estimated_maximum_interval = conf["/**"]["ros__parameters"]["velocity_estimator"]["acceleration_offset"]["estimated_maximum_interval"].as<double>();
    param.buffer_min = param.imu_rate * param.estimated_minimum_interval;
    param.buffer_max = param.imu_rate * param.estimated_maximum_interval;
    param.filter_process_noise = conf["/**"]["ros__parameters"]["velocity_estimator"]["acceleration_offset"]["filter_process_noise"].as<double>();
    param.filter_observation_noise = conf["/**"]["ros__parameters"]["velocity_estimator"]["acceleration_offset"]["filter_observation_noise"].as<double>();

    // std::cout<< "imu_rate "<<param.imu_rate<<std::endl;
    // std::cout<< "gnss_rate "<<param.gnss_rate<<std::endl;
    // std::cout<< "estimated_minimum_interval "<<param.estimated_minimum_interval<<std::endl;
    // std::cout<< "estimated_maximum_interval "<<param.estimated_maximum_interval<<std::endl;
    // std::cout<< "filter_process_noise "<<param.filter_process_noise<<std::endl;
    // std::cout<< "filter_observation_noise "<<param.filter_observation_noise<<std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mAccelerationOffsetEstimator YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
}

bool VelocityEstimator::AccelerationOffsetEstimator::AccelerationOffsetEstimate
(double imu_time_last, double rtkfix_velocity, double pitching,
 double acceleration, bool navsat_update_status)
{
  // Low Path Filter (about acceleration)
  double init_variance = 1.0;
  double acceleration_variance_negative;
  double acceleration_variance_positive;
  double update_gain;

  acceleration_variance_negative = acceleration_variance_last + param.filter_process_noise;
  update_gain = acceleration_variance_negative / (acceleration_variance_negative + param.filter_observation_noise);
  filtered_acceleration = acceleration_last + update_gain * (acceleration - acceleration_last);
  acceleration_variance_positive = (1 - update_gain) * acceleration_variance_negative;

  acceleration_last = filtered_acceleration;
  acceleration_variance_last = acceleration_variance_positive;

  // data buffer generate
  time_buffer.push_back(imu_time_last);
  pitching_buffer.push_back(pitching);
  filtered_acceleration_buffer.push_back(filtered_acceleration);
  rtkfix_velocity_buffer.push_back(rtkfix_velocity);
  navsat_update_status_buffer.push_back(navsat_update_status);

  std::size_t time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());
  if (time_buffer_length < param.buffer_min)
  {
    return acceleration_offset_status.enabled_status;
  } 
  else if (time_buffer_length > param.buffer_max)
  {
    time_buffer.erase(time_buffer.begin());
    pitching_buffer.erase(pitching_buffer.begin());
    filtered_acceleration_buffer.erase(filtered_acceleration_buffer.begin());
    rtkfix_velocity_buffer.erase(rtkfix_velocity_buffer.begin());
    navsat_update_status_buffer.erase(navsat_update_status_buffer.begin());
    time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());
  }

  // Acceleration error estimation by the least-squares method
  double accumulated_acceleration;
  std::vector<double> accumulated_acceleration_buffer;
  std::vector<double> init_accumulated_acceleration_buffer;
  std::vector<int> gnss_index;
  std::size_t gnss_index_length;

  for (int i = 0; i < time_buffer_length; i++)
  {
    if (navsat_update_status_buffer[i]) gnss_index.push_back(i); //TODO Velocity judgment
    if (i == 0)
    {
      accumulated_acceleration_buffer.push_back(0);
      continue;
    }
    else
    {
      accumulated_acceleration += (filtered_acceleration_buffer[i] - g*std::sin(pitching_buffer[i]))* (time_buffer[i]-time_buffer[i-1]);
      accumulated_acceleration_buffer.push_back(accumulated_acceleration);
    }
  }

  gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());

  for (int i = 0; i < time_buffer_length; i++)
  {
    init_accumulated_acceleration_buffer.push_back(accumulated_acceleration_buffer[i] +
        (rtkfix_velocity_buffer[gnss_index[0]] - accumulated_acceleration_buffer[gnss_index[0]]));
  }

  double dt;             // x
  double residual_error; // y
  double sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0;

  for (int i = 0; i < gnss_index_length; i++)
  {
    residual_error = init_accumulated_acceleration_buffer[gnss_index[i]] - rtkfix_velocity_buffer[gnss_index[i]];
    dt = time_buffer[gnss_index[i]]-time_buffer[gnss_index[0]];

    sum_xy += dt * residual_error;
    sum_x  += dt;
    sum_y  += residual_error;
    sum_xx += dt *dt;
  }

  acceleration_offset = -1 * (gnss_index_length*sum_xy - sum_x*sum_y)/(gnss_index_length*sum_xx - sum_x*sum_x);
  acceleration_offset_status.enabled_status = true;
  acceleration_offset_status.estimate_status = true;

  return acceleration_offset_status.enabled_status;
}

//---VelocityEstimator---------------------------------------------------------------------------------------------------------------------
VelocityEstimator::VelocityEstimator()
{
  acceleration = 0;
  pitch_rate = 0;
  doppler_velocity = 0;
  rtkfix_velocity = 0;
  rtkfix_pitching = 0;
  gga_time_last = 0;
  pitch_rate_offset = 0;
  pitching = 0;
  acceleration_offset = 0;
  filtered_acceleration = 0;
  ecef_base_position_status = false;
  velocity_status.enabled_status = false;

}

void VelocityEstimator::setParam(std::string yaml_file)
{
  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    param.ecef_base_pos_x = conf["/**"]["ros__parameters"]["ecef_base_pos"]["x"].as<double>();
    param.ecef_base_pos_y = conf["/**"]["ros__parameters"]["ecef_base_pos"]["y"].as<double>();
    param.ecef_base_pos_z = conf["/**"]["ros__parameters"]["ecef_base_pos"]["z"].as<double>();
    param.use_ecef_base_position = conf["/**"]["ros__parameters"]["ecef_base_pos"]["use_ecef_base_position"].as<bool>();

    param.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    param.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();

    param.gga_downsample_time = conf["/**"]["ros__parameters"]["velocity_estimator"]["gga_downsample_time"].as<double>();
    param.stop_judgment_velocity_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["stop_judgment_velocity_threshold"].as<double>();
    param.stop_judgment_interval = conf["/**"]["ros__parameters"]["velocity_estimator"]["stop_judgment_interval"].as<double>();
    param.stop_judgment_buffer_maxnum =  param.stop_judgment_buffer_maxnum = param.imu_rate * param.stop_judgment_interval;
    param.variance_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["variance_threshold"].as<double>();

    // doppler fusion parameter
    param.estimated_interval = conf["/**"]["ros__parameters"]["velocity_estimator"]["doppler_fusion"]["estimated_interval"].as<double>();
    param.buffer_max = param.imu_rate * param.estimated_interval;
    param.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["doppler_fusion"]["gnss_receiving_threshold"].as<double>();
    param.estimated_gnss_coefficient = param.gnss_rate/param.imu_rate * param.gnss_receiving_threshold;
    param.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["doppler_fusion"]["outlier_ratio_threshold"].as<double>();
    param.estimated_coefficient = param.estimated_gnss_coefficient * param.outlier_ratio_threshold;
    param.outlier_threshold = conf["/**"]["ros__parameters"]["velocity_estimator"]["doppler_fusion"]["outlier_threshold"].as<double>();

    // std::cout<< "imu_rate "<<param.imu_rate<<std::endl;
    // std::cout<< "gnss_rate "<<param.gnss_rate<<std::endl;
    // std::cout<< "ecef_base_pos_x "<<param.ecef_base_pos_x<<std::endl;
    // std::cout<< "ecef_base_pos_y "<<param.ecef_base_pos_y<<std::endl;
    // std::cout<< "ecef_base_pos_z "<<param.ecef_base_pos_z<<std::endl;
    // std::cout<< "use_ecef_base_position "<<param.use_ecef_base_position<<std::endl;
    // std::cout<< "gga_downsample_time "<<param.gga_downsample_time<<std::endl;
    // std::cout<< "stop_judgment_velocity_threshold "<<param.stop_judgment_velocity_threshold<<std::endl;
    // std::cout<< "stop_judgment_interval "<<param.stop_judgment_interval<<std::endl;
    // std::cout<< "stop_judgment_buffer_maxnum "<<param.stop_judgment_buffer_maxnum<<std::endl;
    // std::cout<< "variance_threshold "<<param.variance_threshold<<std::endl;
    // std::cout<< "estimated_interval "<<param.estimated_interval<<std::endl;
    // std::cout<< "gnss_receiving_threshold "<<param.gnss_receiving_threshold<<std::endl;
    // std::cout<< "outlier_ratio_threshold "<<param.outlier_ratio_threshold<<std::endl;
    // std::cout<< "outlier_threshold "<<param.outlier_threshold<<std::endl;

    pitch_rate_offset_stop_estimator.setParam(yaml_file);
    pitching_estimator.setParam(yaml_file);
    acceleration_offset_estimator.setParam(yaml_file);
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mVelocityEstimator YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
}

eagleye_msgs::msg::Status VelocityEstimator::getStatus()
{
  eagleye_msgs::msg::Status status;
  status.enabled_status = velocity_status.enabled_status;
  status.estimate_status = velocity_status.estimate_status;

  return status;
}

bool VelocityEstimator::updateImu(sensor_msgs::msg::Imu imu_msg)
{
  rclcpp::Time ros_clock(imu_msg.header.stamp);
  double imu_time = ros_clock.seconds();
  if(imu_time == 0) return false;
  if(imu_time == imu_time_last) return false;

  acceleration = imu_msg.linear_acceleration.x;
  pitch_rate = imu_msg.angular_velocity.y;
  imu_time_last = imu_time;

  return true;
}

bool VelocityEstimator::updateRtklibNav(rtklib_msgs::msg::RtklibNav rtklib_nav_msg)
{
  rclcpp::Time ros_clock(rtklib_nav_msg.header.stamp);
  double rtklib_nav_time = ros_clock.seconds();
  if(rtklib_nav_time == 0) return false;
  if(rtklib_nav_time == rtklib_nav_time_last) return false;
  rtklib_nav_time_last = rtklib_nav_time;

  double rtklib_position_ecef[3];
  double rtklib_velocity_ecef[3];
  double rtklib_velocity_enu[3];

  bool rtklib_transformation_failure;

  rtklib_position_ecef[0] = rtklib_nav_msg.ecef_pos.x;
  rtklib_position_ecef[1] = rtklib_nav_msg.ecef_pos.y;
  rtklib_position_ecef[2] = rtklib_nav_msg.ecef_pos.z;
  rtklib_velocity_ecef[0] = rtklib_nav_msg.ecef_vel.x;
  rtklib_velocity_ecef[1] = rtklib_nav_msg.ecef_vel.y;
  rtklib_velocity_ecef[2] = rtklib_nav_msg.ecef_vel.z;
  xyz2enu_vel(rtklib_velocity_ecef, rtklib_position_ecef, rtklib_velocity_enu);

  if (!std::isfinite(rtklib_velocity_enu[0])||!std::isfinite(rtklib_velocity_enu[1])||!std::isfinite(rtklib_velocity_enu[2]))
  {
    rtklib_transformation_failure = true;
    return false;
  }
  else
  {
    doppler_velocity = std::sqrt(
      (rtklib_velocity_enu[0]*rtklib_velocity_enu[0]) 
      +(rtklib_velocity_enu[1]*rtklib_velocity_enu[1]) 
      +(rtklib_velocity_enu[2]*rtklib_velocity_enu[2]));
    rtklib_transformation_failure = false;
  }

  return true;
}

bool VelocityEstimator::updateGGA(nmea_msgs::msg::Gpgga gga_msg)
{
  rclcpp::Time ros_clock(gga_msg.header.stamp);
  double gga_time = ros_clock.seconds();
  if(gga_time == 0) return false;
  if(gga_time == gga_time_last) return false;
  if(std::abs(gga_time - gga_time_last) < param.gga_downsample_time) return false;

  double gga_position_llh[3];
  double gga_position_ecef[3];
  double gga_position_enu[3];

  if(param.use_ecef_base_position && !ecef_base_position_status)
  {
    ecef_base_position[0] = param.ecef_base_pos_x;
    ecef_base_position[1] = param.ecef_base_pos_y;
    ecef_base_position[2] = param.ecef_base_pos_z;
    ecef_base_position_status = true;
  }
  else if(!ecef_base_position_status && gga_time != 0 &&
          gga_msg.lat != 0 && gga_msg.lon != 0 && gga_msg.alt + gga_msg.undulation != 0)
  {
    gga_position_llh[0] = gga_msg.lat *M_PI/180;
    gga_position_llh[1] = gga_msg.lon *M_PI/180;
    gga_position_llh[2] = gga_msg.alt + gga_msg.undulation;
    llh2xyz(gga_position_llh, ecef_base_position);
    ecef_base_position_status = true;
  }

  gga_position_llh[0] = gga_msg.lat *M_PI/180;
  gga_position_llh[1] = gga_msg.lon *M_PI/180;
  gga_position_llh[2] = gga_msg.alt + gga_msg.undulation;
  llh2xyz(gga_position_llh, gga_position_ecef);
  xyz2enu(gga_position_ecef, ecef_base_position, gga_position_enu);

  double difference_time;
  double difference_position[3];
  double horizontal_velocity;
  double vertical_velocity;

  if(gga_msg.gps_qual == STATUS_FIX && gga_status_last == STATUS_FIX
     && gga_time != gga_time_last
     && gga_time != 0)
  {
    difference_time = gga_time - gga_time_last;
    difference_position[0] = gga_position_enu[0] - gga_position_enu_last[0];
    difference_position[1] = gga_position_enu[1] - gga_position_enu_last[1];
    difference_position[2] = gga_position_enu[2] - gga_position_enu_last[2];

    rtkfix_velocity = std::sqrt(difference_position[0]*difference_position[0] + difference_position[1]*difference_position[1] +
        difference_position[2]*difference_position[2])/difference_time;
    horizontal_velocity = std::sqrt(difference_position[0]*difference_position[0] + difference_position[1]*difference_position[1]) / difference_time;
    vertical_velocity = difference_position[2]/difference_time;
    rtkfix_pitching = std::atan2(vertical_velocity,horizontal_velocity);
  }

  gga_time_last = gga_time;
  memcpy(gga_position_enu_last, gga_position_enu, sizeof(gga_position_enu));
  gga_status_last = gga_msg.gps_qual;

  return true;
}

bool VelocityEstimator::StopJudgment(sensor_msgs::msg::Imu imu_msg)
{
  angular_velocity_x_buffer.push_back(imu_msg.angular_velocity.x);
  angular_velocity_y_buffer.push_back(imu_msg.angular_velocity.y);
  angular_velocity_z_buffer.push_back(imu_msg.angular_velocity.z);

  std::size_t buffer_length = std::distance(angular_velocity_x_buffer.begin(), angular_velocity_x_buffer.end());
  if (buffer_length <= param.stop_judgment_buffer_maxnum) return false;

  angular_velocity_x_buffer.erase(angular_velocity_x_buffer.begin());
  angular_velocity_y_buffer.erase(angular_velocity_y_buffer.begin());
  angular_velocity_z_buffer.erase(angular_velocity_z_buffer.begin());

  // ToDo: GNSS time update check
  if (doppler_velocity > param.stop_judgment_velocity_threshold ||
      rtkfix_velocity > param.stop_judgment_velocity_threshold)
  {return false;}

  double sum_x = 0, sum_y = 0, sum_z = 0;
  double var_x = 0, var_y = 0, var_z = 0;

  for (int i = 0; i < buffer_length; i++)
  {
    sum_x = sum_x + std::pow(angular_velocity_x_buffer[i] - std::accumulate(std::begin(angular_velocity_x_buffer),
        std::end(angular_velocity_x_buffer), 0.0)/buffer_length,2.0);
    sum_y = sum_y + std::pow(angular_velocity_y_buffer[i] - std::accumulate(std::begin(angular_velocity_y_buffer),
        std::end(angular_velocity_y_buffer), 0.0)/buffer_length,2.0);
    sum_z = sum_z + std::pow(angular_velocity_z_buffer[i] - std::accumulate(std::begin(angular_velocity_z_buffer),
        std::end(angular_velocity_z_buffer), 0.0)/buffer_length,2.0);
  }

  var_x = sum_x/buffer_length;
  var_y = sum_y/buffer_length;
  var_z = sum_z/buffer_length;

  if (var_x > param.variance_threshold || var_y > param.variance_threshold || var_z > param.variance_threshold)
  {return false;}

  return true;
}

bool VelocityEstimator::DopplerFusion()
{
  // data buffer generate
  time_buffer.push_back(imu_time_last);
  doppler_velocity_buffer.push_back(doppler_velocity);
  corrected_acceleration_buffer.push_back(filtered_acceleration + acceleration_offset - g*std::sin(pitching));
  rtklib_update_status_buffer.push_back(rtklib_update_status);

  std::size_t time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());
  if (time_buffer_length <= param.buffer_max) return velocity_status.enabled_status;

  time_buffer.erase(time_buffer.begin());
  doppler_velocity_buffer.erase(doppler_velocity_buffer.begin());
  corrected_acceleration_buffer.erase(corrected_acceleration_buffer.begin());
  rtklib_update_status_buffer.erase(rtklib_update_status_buffer.begin());
  time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());

  // stop process
  if (stop_status && velocity_status.enabled_status)
  {
    velocity = 0;
    velocity_status.enabled_status = true;
    velocity_status.estimate_status = true;

    return velocity_status.enabled_status;
  }

  // setup for estimation
  double accumulated_acceleration;
  std::vector<double> accumulated_acceleration_buffer(time_buffer_length);
  std::vector<int> gnss_index;
  std::size_t gnss_index_length;

  for (int i = 0; i < time_buffer_length; i++)
  {
    if (rtklib_update_status_buffer[i] && doppler_velocity_buffer[i] < 33.3) gnss_index.push_back(i); //TODO Velocity judgment
    if (i == 0) continue;
    accumulated_acceleration += corrected_acceleration_buffer[i] * (time_buffer[i]-time_buffer[i-1]);
    accumulated_acceleration_buffer[i] = accumulated_acceleration;
  }

  gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());
  if (gnss_index_length < time_buffer_length * param.estimated_gnss_coefficient) return velocity_status.enabled_status;

  // velocity estimation by the least-squares method
  std::vector<double> init_accumulated_acceleration;
  std::vector<double> residual_error;

  while (1)
  {
    gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());

    init_accumulated_acceleration.clear();
    for (int i = 0; i < time_buffer_length; i++)
    {
      init_accumulated_acceleration.push_back(accumulated_acceleration_buffer[i] +
        (doppler_velocity_buffer[gnss_index[0]] - accumulated_acceleration_buffer[gnss_index[0]]));
    }

    residual_error.clear();
    for (int i = 0; i < gnss_index_length; i++)
    {
      residual_error.push_back(init_accumulated_acceleration[gnss_index[i]] - doppler_velocity_buffer[gnss_index[i]]);
    }

    double residual_error_mean = std::accumulate(residual_error.begin(), residual_error.end(), 0.0) / gnss_index_length;
    double init_velocity = doppler_velocity_buffer[gnss_index[0]] - residual_error_mean;

    init_accumulated_acceleration.clear();
    for (int i = 0; i < time_buffer_length; i++)
    {
      init_accumulated_acceleration.push_back(accumulated_acceleration_buffer[i] + (init_velocity - accumulated_acceleration_buffer[gnss_index[0]]));
    }

    residual_error.clear();
    for (int i = 0; i < gnss_index_length; i++)
    {
      residual_error.push_back(init_accumulated_acceleration[gnss_index[i]] - doppler_velocity_buffer[gnss_index[i]]);
    }

    std::vector<double>::iterator residual_error_max = std::max_element(residual_error.begin(), residual_error.end());
    int residual_error_max_index = std::distance(residual_error.begin(), residual_error_max);

    if (residual_error[residual_error_max_index] < param.outlier_threshold) break;
    gnss_index.erase(gnss_index.begin() + residual_error_max_index);
    gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());
    if (gnss_index_length < param.buffer_max  * param.estimated_coefficient) break;

  }

  gnss_index_length = std::distance(gnss_index.begin(), gnss_index.end());

  velocity = init_accumulated_acceleration[time_buffer_length-1];
  velocity_status.enabled_status = true;
  velocity_status.estimate_status = true;

  return velocity_status.enabled_status;

}

void VelocityEstimator::VelocityEstimate(sensor_msgs::msg::Imu imu_msg, rtklib_msgs::msg::RtklibNav rtklib_nav_msg, nmea_msgs::msg::Gpgga gga_msg, geometry_msgs::msg::TwistStamped* velocity_msg)
{
  // imu msgs setting
  updateImu(imu_msg);

  // rtklib_nav msgs setting
  rtklib_update_status = updateRtklibNav(rtklib_nav_msg);

  // gga msgs setting
  navsat_update_status = updateGGA(gga_msg);

  // Stop Judgment
  stop_status = StopJudgment(imu_msg);
  velocity_status.estimate_status = false;

  // pitch_rate offset estimation during stop
  pitch_rate_offset_stop_estimator.PitchrateOffsetStopEstimate(pitch_rate, stop_status);
  pitch_rate_offset = pitch_rate_offset_stop_estimator.pitch_rate_offset;

  if(!pitching_estimator.PitchingEstimate
      (imu_time_last, doppler_velocity, rtkfix_velocity,
       pitch_rate, pitch_rate_offset, rtkfix_pitching,
       navsat_update_status, stop_status)
    ) return;
  pitching = pitching_estimator.pitching;

  // Acceleration Error Estimation
  if(!acceleration_offset_estimator.AccelerationOffsetEstimate
      (imu_time_last, rtkfix_velocity, pitching,
       acceleration, navsat_update_status)
    ) return ;
  acceleration_offset = acceleration_offset_estimator.acceleration_offset;
  filtered_acceleration = acceleration_offset_estimator.filtered_acceleration;

  // Velocity estimation by fusion of acceleration accumulation and GNSS Doppler
  if(!DopplerFusion()) return;

  velocity_msg->header = imu_msg.header;
  velocity_msg->header.frame_id = "base_link";
  velocity_msg->twist.linear.x = velocity;
} 