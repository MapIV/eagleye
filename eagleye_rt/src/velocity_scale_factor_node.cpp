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
 * velocity_scale_factor.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static rtklib_msgs::RtklibNav rtklib_nav;
static nmea_msgs::Gprmc nmea_rmc;
static geometry_msgs::TwistStamped velocity;
static sensor_msgs::Imu imu;

static ros::Publisher pub;
static ros::Timer timer;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;

struct VelocityScaleFactorParameter velocity_scale_factor_parameter;
struct VelocityScaleFactorStatus velocity_scale_factor_status;

static std::string use_gnss_mode;

bool is_first_move = false;

std::string velocity_scale_factor_save_str;
double saved_vsf_estimater_number;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

void rmc_callback(const nmea_msgs::Gprmc::ConstPtr& msg)
{
  nmea_rmc = *msg;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  velocity.header = msg->header;
  velocity.twist = msg->twist;

  if (is_first_move == false && msg->twist.linear.x > velocity_scale_factor_parameter.estimated_velocity_threshold)
  {
    is_first_move = true;
  }
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double initial_velocity_scale_factor = 1.0;

  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.header.frame_id = "base_link";

  if (is_first_move == false)
  {
    velocity_scale_factor.scale_factor = initial_velocity_scale_factor;
    velocity_scale_factor.correction_velocity = velocity.twist;
    velocity_scale_factor.status.enabled_status = false;
    velocity_scale_factor.status.estimate_status = false;
    pub.publish(velocity_scale_factor);
    return;
  }

  if (use_gnss_mode == "rtklib" || use_gnss_mode == "RTKLIB") // use RTKLIB mode
    velocity_scale_factor_estimate(rtklib_nav,velocity,velocity_scale_factor_parameter,&velocity_scale_factor_status,&velocity_scale_factor);
  else if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
    velocity_scale_factor_estimate(nmea_rmc,velocity,velocity_scale_factor_parameter,&velocity_scale_factor_status,&velocity_scale_factor);
  pub.publish(velocity_scale_factor);
}

void load_velocity_scale_factor(std::string txt_path)
{
  std::ifstream ifs(txt_path);
  if (!ifs)
  {
    std::cout << "Initial VelocityScaleFactor file not found!!" << std::endl;
  }
  else
  {
    int count = 0;
    std::string row;
    while (getline(ifs, row))
    {
      if(count == 1)
      {
        saved_vsf_estimater_number = std::stod(row);
      }
      if(count == 3)
      {
        double saved_velocity_scale_factor = std::stod(row);
        velocity_scale_factor_status.velocity_scale_factor_last = saved_velocity_scale_factor;
        velocity_scale_factor.scale_factor = saved_velocity_scale_factor;
      }
      count++;
    }
  }
}

void timer_callback(const ros::TimerEvent & e)
{
  if(!velocity_scale_factor.status.enabled_status && saved_vsf_estimater_number >= velocity_scale_factor_status.estimated_number)
  {
    return;
  }

  std::ofstream csv_file(velocity_scale_factor_save_str);
  csv_file << "estimated_number";
  csv_file << "\n";
  csv_file << velocity_scale_factor_status.estimated_number;
  csv_file << "\n";
  csv_file << "velocity_scale_factor";
  csv_file << "\n";
  csv_file << velocity_scale_factor_status.velocity_scale_factor_last;
  csv_file << "\n";

  saved_vsf_estimater_number = velocity_scale_factor_status.estimated_number;

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_scale_factor");
  ros::NodeHandle n;

  std::string subscribe_twist_topic_name = "/can_twist";
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";
  std::string subscribe_rmc_topic_name = "/mosaic/rmc";

  double velocity_scale_factor_save_duration; // [sec]

  n.getParam("twist_topic",subscribe_twist_topic_name);
  n.getParam("imu_topic",subscribe_imu_topic_name);
  n.getParam("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("rmc_topic",subscribe_rmc_topic_name);
  n.getParam("velocity_scale_factor/estimated_number_min",velocity_scale_factor_parameter.estimated_number_min);
  n.getParam("velocity_scale_factor/estimated_number_max",velocity_scale_factor_parameter.estimated_number_max);
  n.getParam("velocity_scale_factor/estimated_velocity_threshold",velocity_scale_factor_parameter.estimated_velocity_threshold);
  n.getParam("velocity_scale_factor/estimated_coefficient",velocity_scale_factor_parameter.estimated_coefficient);
  n.getParam("use_gnss_mode",use_gnss_mode);
  n.getParam("velocity_scale_factor_save_str", velocity_scale_factor_save_str);
  n.getParam("velocity_scale_factor/save_velocity_scale_factor", velocity_scale_factor_parameter.save_velocity_scale_factor);
  n.getParam("velocity_scale_factor/velocity_scale_factor_save_duration", velocity_scale_factor_save_duration);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "subscribe_rmc_topic_name "<<subscribe_rmc_topic_name<<std::endl;
  std::cout<< "estimated_number_min "<<velocity_scale_factor_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<velocity_scale_factor_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<velocity_scale_factor_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_coefficient "<<velocity_scale_factor_parameter.estimated_coefficient<<std::endl;
  std::cout<< "use_gnss_mode "<<use_gnss_mode<<std::endl;
  std::cout<< "velocity_scale_factor_save_str "<<velocity_scale_factor_save_str<<std::endl;
  std::cout<< "save_velocity_scale_factor "<<velocity_scale_factor_parameter.save_velocity_scale_factor<<std::endl;
  std::cout<< "velocity_scale_factor_save_duration "<<velocity_scale_factor_save_duration<<std::endl;

  ros::Subscriber sub1 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe(subscribe_rmc_topic_name, 1000, rmc_callback, ros::TransportHints().tcpNoDelay());
  pub = n.advertise<eagleye_msgs::VelocityScaleFactor>("velocity_scale_factor", 1000);
  if(velocity_scale_factor_parameter.save_velocity_scale_factor)
  {
    timer = n.createTimer(ros::Duration(velocity_scale_factor_save_duration), timer_callback);

    load_velocity_scale_factor(velocity_scale_factor_save_str);
  }

  ros::spin();

  return 0;
}
