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

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

class TFConvertedIMU: public rclcpp::Node
{
public:
  TFConvertedIMU();
  ~TFConvertedIMU();

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::Imu tf_converted_imu_;

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  std::string tf_base_link_frame_;

  bool reverse_imu_wz_;

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

};

TFConvertedIMU::TFConvertedIMU() : Node("eagleye_tf_converted_imu"),
    logger_(get_logger()),
    clock_(RCL_ROS_TIME),
    tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
    tflistener_(tfbuffer_)
{
  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string publish_imu_topic_name = "imu/data_tf_converted";

  declare_parameter("imu_topic", subscribe_imu_topic_name);
  declare_parameter("publish_imu_topic", publish_imu_topic_name);
  declare_parameter("tf_gnss_frame.parent", tf_base_link_frame_);
  declare_parameter("reverse_imu_wz", reverse_imu_wz_);

  get_parameter("imu_topic", subscribe_imu_topic_name);
  get_parameter("publish_imu_topic", publish_imu_topic_name);
  get_parameter("tf_gnss_frame.parent", tf_base_link_frame_);
  get_parameter("reverse_imu_wz", reverse_imu_wz_);

  std::cout<< "subscribe_imu_topic_name: " << subscribe_imu_topic_name << std::endl;
  std::cout<< "publish_imu_topic_name: " << publish_imu_topic_name << std::endl;
  std::cout<< "tf_base_link_frame: " << tf_base_link_frame_ << std::endl;
  std::cout<< "reverse_imu_wz: " << reverse_imu_wz_ << std::endl;

  sub_ =  create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, rclcpp::QoS(10), std::bind(&TFConvertedIMU::imu_callback, this, std::placeholders::_1));
  pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_tf_converted", rclcpp::QoS(10));
};

TFConvertedIMU::~TFConvertedIMU(){}; 

void TFConvertedIMU::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu_ = *msg;
  tf_converted_imu_.header = imu_.header;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     tf_base_link_frame_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = imu_.header;
    angular_velocity.vector = imu_.angular_velocity;
    linear_acceleration.header = imu_.header;
    linear_acceleration.vector = imu_.linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu_.angular_velocity = transformed_angular_velocity.vector;
    if(reverse_imu_wz_)
    {
      tf_converted_imu_.angular_velocity.z = (-1) * transformed_angular_velocity.vector.z;
    }
    else
    {
      tf_converted_imu_.angular_velocity.z = transformed_angular_velocity.vector.z;
    }
    tf_converted_imu_.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu_.orientation = transformed_quaternion;

  } 
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(rclcpp::get_logger("tf_converted_imu"), "Failed to lookup transform.");
    return;
  }
  pub_->publish(tf_converted_imu_);
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TFConvertedIMU>());

  return 0;
}
