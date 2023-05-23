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

/*
 * correction_imu.cpp
 * Author MapIV Sasaki
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

class TFConvertedIMU
{
public:
  TFConvertedIMU();
  ~TFConvertedIMU();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  sensor_msgs::Imu imu_;
  sensor_msgs::Imu tf_converted_imu_;

  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  std::string tf_base_link_frame_;

  bool reverse_imu_wz_, reverse_imu_ax_;

  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

};

TFConvertedIMU::TFConvertedIMU() : tflistener_(tfbuffer_)
{
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string publish_imu_topic_name = "imu/data_tf_converted";

  std::string yaml_file;
  nh_.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    subscribe_imu_topic_name = conf["imu_topic"].as<std::string>();
    tf_base_link_frame_ = conf["tf_gnss_frame"]["parent"].as<std::string>();
    reverse_imu_wz_ = conf["reverse_imu_wz"].as<bool>();
    reverse_imu_ax_ = conf["reverse_imu_ax"].as<bool>();
    std::cout<< "subscribe_imu_topic_name: " << subscribe_imu_topic_name << std::endl;
    std::cout<< "publish_imu_topic_name: " << publish_imu_topic_name << std::endl;
    std::cout<< "tf_base_link_frame: " << tf_base_link_frame_ << std::endl;

  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mtf_converted_imu Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  sub_ = nh_.subscribe(subscribe_imu_topic_name, 1000, &TFConvertedIMU::imu_callback, this, ros::TransportHints().tcpNoDelay());
  pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_tf_converted", 1000);
};

TFConvertedIMU::~TFConvertedIMU(){};

void TFConvertedIMU::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_ = *msg;
  tf_converted_imu_.header = imu_.header;

  try {
    const geometry_msgs::TransformStamped transform = tfbuffer_.lookupTransform(
      tf_base_link_frame_, msg->header.frame_id, ros::Time(0));

    geometry_msgs::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::Quaternion  transformed_quaternion;

    angular_velocity.header = imu_.header;
    angular_velocity.vector = imu_.angular_velocity;
    linear_acceleration.header = imu_.header;
    linear_acceleration.vector = imu_.linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);
    tf2::doTransform(imu_.orientation, transformed_quaternion, transform);

    tf_converted_imu_.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu_.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu_.orientation = transformed_quaternion;

    if(reverse_imu_wz_)
    {
      tf_converted_imu_.angular_velocity.z = (-1) * transformed_angular_velocity.vector.z;
    }
    if(reverse_imu_ax_)
    {
      tf_converted_imu_.linear_acceleration.x = (-1) * transformed_linear_acceleration.vector.x;
    }

  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Failed to lookup transform: %s", ex.what());
    return;
  }
  pub_.publish(tf_converted_imu_);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_converted_imu");

  TFConvertedIMU main_node;

  ros::spin();

  return 0;
}
