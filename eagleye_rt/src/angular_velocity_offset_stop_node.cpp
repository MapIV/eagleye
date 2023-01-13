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

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static geometry_msgs::msg::TwistStamped velocity;
rclcpp::Publisher<eagleye_msgs::msg::AngularVelocityOffset>::SharedPtr pub;
static eagleye_msgs::msg::AngularVelocityOffset angular_velocity_offset_stop;
static sensor_msgs::msg::Imu imu;

struct AngularVelocityOffsetStopParameter angular_velocity_offset_stop_parameter;
struct AngularVelocityOffsetStopStatus angular_velocity_offset_stop_status;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  angular_velocity_offset_stop.header = msg->header;
  angular_velocity_offset_stop_estimate(velocity, imu, angular_velocity_offset_stop_parameter, &angular_velocity_offset_stop_status, &angular_velocity_offset_stop);
  pub->publish(angular_velocity_offset_stop);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_angular_velocity_offset_stop");

  std::string subscribe_twist_topic_name = "vehicle/twist";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

 try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    angular_velocity_offset_stop_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    angular_velocity_offset_stop_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    angular_velocity_offset_stop_parameter.estimated_interval = conf["/**"]["ros__parameters"]["angular_velocity_offset_stop"]["estimated_interval"].as<double>();
    angular_velocity_offset_stop_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["angular_velocity_offset_stop"]["outlier_threshold"].as<double>();

    std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;
    std::cout << "imu_rate " << angular_velocity_offset_stop_parameter.imu_rate << std::endl;
    std::cout << "stop_judgment_threshold " << angular_velocity_offset_stop_parameter.stop_judgment_threshold << std::endl;
    std::cout << "estimated_minimum_interval " << angular_velocity_offset_stop_parameter.estimated_interval << std::endl;
    std::cout << "outlier_threshold " << angular_velocity_offset_stop_parameter.outlier_threshold << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mangular_velocity_offset_stop Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  auto sub1 = node->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, velocity_callback);
  auto sub2 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  pub = node->create_publisher<eagleye_msgs::msg::AngularVelocityOffset>("angular_velocity_offset_stop", 1000);

  rclcpp::spin(node);

  return 0;
}
