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

static ros::Publisher _pub;
static sensor_msgs::Imu _imu;

sensor_msgs::Imu _tf_converted_imu;

std::string _tf_base_link_frame;

tf2_ros::Buffer tfbuffer_;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_msgs::Imu tf_converted_imu;

  _imu = *msg;
  _tf_converted_imu.header = _imu.header;
  _tf_converted_imu.orientation = _imu.orientation;
  _tf_converted_imu.orientation_covariance = _imu.orientation_covariance;
  _tf_converted_imu.angular_velocity_covariance = _imu.angular_velocity_covariance;
  _tf_converted_imu.linear_acceleration_covariance = _imu.linear_acceleration_covariance;

  try {
    const geometry_msgs::TransformStamped transform = tfbuffer_.lookupTransform(
      _tf_base_link_frame, msg->header.frame_id, msg->header.stamp);
    tf2::doTransform(*msg, tf_converted_imu, transform);
  } 
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  _pub.publish(_tf_converted_imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "correction_imu");
  ros::NodeHandle nh;
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string publish_imu_topic_name = "/imu/data_tf_converted";

  nh.getParam("imu_topic", subscribe_imu_topic_name);
  nh.getParam("publish_imu_topic", publish_imu_topic_name);
  nh.getParam("tf_gnss_frame/parent", _tf_base_link_frame);
  std::cout<< "subscribe_imu_topic_name: " << subscribe_imu_topic_name << std::endl;
  std::cout<< "publish_imu_topic_name: " << publish_imu_topic_name << std::endl;
  std::cout<< "tf_base_link_frame: " << _tf_base_link_frame << std::endl;

  ros::Subscriber sub5 = nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  _pub = nh.advertise<sensor_msgs::Imu>(publish_imu_topic_name, 1000);

  ros::spin();

  return 0;
}
