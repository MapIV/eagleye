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
 * angular_velocity_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static geometry_msgs::TwistStamped _velocity;
static ros::Publisher _pub;
static eagleye_msgs::AngularVelocityOffset _angular_velocity_offset_stop;
static sensor_msgs::Imu _imu;

struct AngularVelocityOffsetStopParameter _angular_velocity_offset_stop_parameter;
struct AngularVelocityOffsetStopStatus _angular_velocity_offset_stop_status;

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  _velocity = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  _imu = *msg;
  _angular_velocity_offset_stop.header = msg->header;
  angular_velocity_offset_stop_estimate(_velocity, _imu, _angular_velocity_offset_stop_parameter,
    &_angular_velocity_offset_stop_status, &_angular_velocity_offset_stop);
  _pub.publish(_angular_velocity_offset_stop);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "angular_velocity_offset_stop");
  ros::NodeHandle nh;

  std::string subscribe_twist_topic_name = "vehicle/twist";

  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _angular_velocity_offset_stop_parameter.imu_rate = conf["common"]["imu_rate"].as<double>();
    _angular_velocity_offset_stop_parameter.stop_judgment_threshold = conf["common"]["stop_judgment_threshold"].as<double>();
    _angular_velocity_offset_stop_parameter.estimated_interval = conf["angular_velocity_offset_stop"]["estimated_interval"].as<double>();
    _angular_velocity_offset_stop_parameter.outlier_threshold = conf["angular_velocity_offset_stop"]["outlier_threshold"].as<double>();

    std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;
    std::cout << "imu_rate " << _angular_velocity_offset_stop_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << _angular_velocity_offset_stop_parameter.stop_judgment_threshold << std::endl;
    std::cout << "estimated_minimum_interval " << _angular_velocity_offset_stop_parameter.estimated_interval << std::endl;
    std::cout << "outlier_threshold " << _angular_velocity_offset_stop_parameter.outlier_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mangular_velocity_offset_stop Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  ros::Subscriber sub1 = nh.subscribe(subscribe_twist_topic_name, 1000, velocity_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("imu/data_tf_converted", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<eagleye_msgs::AngularVelocityOffset>("angular_velocity_offset_stop", 1000);

  ros::spin();

  return 0;
}
