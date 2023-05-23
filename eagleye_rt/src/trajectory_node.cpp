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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::Imu _imu;
static geometry_msgs::TwistStamped _velocity;
static eagleye_msgs::StatusStamped _velocity_status;
static geometry_msgs::TwistStamped _correction_velocity;
static eagleye_msgs::VelocityScaleFactor _velocity_scale_factor;
static eagleye_msgs::Heading _heading_interpolate_3rd;
static eagleye_msgs::YawrateOffset _yaw_rate_offset_stop;
static eagleye_msgs::YawrateOffset _yaw_rate_offset_2nd;
static eagleye_msgs::Pitching _pitching;

static geometry_msgs::Vector3Stamped _enu_vel;
static eagleye_msgs::Position _enu_relative_pos;
static geometry_msgs::TwistStamped _eagleye_twist;
static geometry_msgs::TwistWithCovarianceStamped _eagleye_twist_with_covariance;
static ros::Publisher _pub1;
static ros::Publisher _pub2;
static ros::Publisher _pub3;
static ros::Publisher _pub4;

struct TrajectoryParameter _trajectory_parameter;
struct TrajectoryStatus _trajectory_status;

static double _timer_updata_rate = 10;
static double _deadlock_threshold = 1;

static double _imu_time_last, _velocity_time_last;
static bool _input_status;

static bool _use_can_less_mode;


void correction_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _correction_velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::StatusStamped::ConstPtr& msg)
{
  _velocity_status = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  _velocity_scale_factor = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_3rd = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset_stop = *msg;
}

void yaw_rate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  _yaw_rate_offset_2nd = *msg;
}

void pitching_callback(const eagleye_msgs::Pitching::ConstPtr& msg)
{
  _pitching = *msg;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _velocity = *msg;
}

void timer_callback(const ros::TimerEvent& e)
{
  if (std::abs(_imu.header.stamp.toSec() - _imu_time_last) < _deadlock_threshold &&
      std::abs(_velocity.header.stamp.toSec() - _velocity_time_last) < _deadlock_threshold &&
      std::abs(_velocity.header.stamp.toSec() - _imu.header.stamp.toSec()) < _deadlock_threshold)
  {
    _input_status = true;
  }
  else
  {
    _input_status = false;
    ROS_WARN("Twist is missing the required input topics.");
  }
  
  if (_imu.header.stamp.toSec() != _imu_time_last) _imu_time_last = _imu.header.stamp.toSec();
  if (_velocity.header.stamp.toSec() != _velocity_time_last) _velocity_time_last = _velocity.header.stamp.toSec();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if(_use_can_less_mode && !_velocity_status.status.enabled_status) return;

  eagleye_msgs::StatusStamped velocity_enable_status;
  if(_use_can_less_mode)
  {
    velocity_enable_status = _velocity_status;
  }
  else
  {
    velocity_enable_status.header = _velocity_scale_factor.header;
    velocity_enable_status.status = _velocity_scale_factor.status;
  }

  _imu = *msg;
  if(_input_status)
  {
    _enu_vel.header = msg->header;
    _enu_vel.header.frame_id = "gnss";
    _enu_relative_pos.header = msg->header;
    _enu_relative_pos.header.frame_id = "base_link";
    _eagleye_twist.header = msg->header;
    _eagleye_twist.header.frame_id = "base_link";
    trajectory3d_estimate(_imu, _correction_velocity, velocity_enable_status, _heading_interpolate_3rd, _yaw_rate_offset_stop, 
      _yaw_rate_offset_2nd, _pitching, _trajectory_parameter, &_trajectory_status, &_enu_vel, &_enu_relative_pos, &_eagleye_twist,
      &_eagleye_twist_with_covariance);

    if(_heading_interpolate_3rd.status.enabled_status)
    {
      _pub1.publish(_enu_vel);
      _pub2.publish(_enu_relative_pos);
    }
    _pub3.publish(_eagleye_twist);
    _pub4.publish(_eagleye_twist_with_covariance);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");

  ros::NodeHandle nh;

  std::string subscribe_twist_topic_name = "vehicle/twist";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_can_less_mode = conf["use_can_less_mode"].as<bool>();

    _trajectory_parameter.stop_judgement_threshold = conf["common"]["stop_judgement_threshold"].as<double>();
    _trajectory_parameter.curve_judgement_threshold = conf["trajectory"]["curve_judgement_threshold"].as<double>();

    _trajectory_parameter.sensor_noise_velocity = conf["trajectory"]["sensor_noise_velocity"].as<double>();
    _trajectory_parameter.sensor_scale_noise_velocity = conf["trajectory"]["sensor_scale_noise_velocity"].as<double>();
    _trajectory_parameter.sensor_noise_yaw_rate = conf["trajectory"]["sensor_noise_yaw_rate"].as<double>();
    _trajectory_parameter.sensor_bias_noise_yaw_rate = conf["trajectory"]["sensor_bias_noise_yaw_rate"].as<double>();

    _timer_updata_rate = conf["trajectory"]["timer_updata_rate"].as<double>();
    _deadlock_threshold = conf["trajectory"]["deadlock_threshold"].as<double>();

    std::cout<< "use_can_less_mode " << _use_can_less_mode << std::endl;

    std::cout<< "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;

    std::cout << "stop_judgement_threshold " << _trajectory_parameter.stop_judgement_threshold << std::endl;
    std::cout << "curve_judgement_threshold " << _trajectory_parameter.curve_judgement_threshold << std::endl;

    std::cout << "sensor_noise_velocity " << _trajectory_parameter.sensor_noise_velocity << std::endl;
    std::cout << "sensor_scale_noise_velocity " << _trajectory_parameter.sensor_scale_noise_velocity << std::endl;
    std::cout << "sensor_noise_yaw_rate " << _trajectory_parameter.sensor_noise_yaw_rate << std::endl;
    std::cout << "sensor_bias_noise_yaw_rate " << _trajectory_parameter.sensor_bias_noise_yaw_rate << std::endl;

    std::cout << "timer_updata_rate " << _timer_updata_rate << std::endl;
    std::cout << "deadlock_threshold " << _deadlock_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mtrajectory Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
  
  ros::Subscriber sub1 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("velocity", 1000, correction_velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("velocity_status", 1000, velocity_status_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = nh.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = nh.subscribe("yaw_rate_offset_stop", 1000, yaw_rate_offset_stop_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub8 = nh.subscribe("yaw_rate_offset_2nd", 1000, yaw_rate_offset_2nd_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub9 = nh.subscribe("pitching", 1000, pitching_callback, ros::TransportHints().tcpNoDelay());
  _pub1 = nh.advertise<geometry_msgs::Vector3Stamped>("enu_vel", 1000);
  _pub2 = nh.advertise<eagleye_msgs::Position>("enu_relative_pos", 1000);
  _pub3 = nh.advertise<geometry_msgs::TwistStamped>("twist", 1000);
  _pub4 = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist_with_covariance", 1000);

  ros::Timer timer = nh.createTimer(ros::Duration(1/_timer_updata_rate), timer_callback);

  ros::spin();

  return 0;
}
