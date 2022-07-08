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
 * yawrate_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static geometry_msgs::msg::TwistStamped velocity;
rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static sensor_msgs::msg::Imu imu;

struct YawrateOffsetStopParameter yawrate_offset_stop_parameter;
struct YawrateOffsetStopStatus yawrate_offset_stop_status;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imu = *msg;
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop_estimate(velocity, imu, yawrate_offset_stop_parameter, &yawrate_offset_stop_status, &yawrate_offset_stop);
  pub->publish(yawrate_offset_stop);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yawrate_offset_stop");
  std::string subscribe_twist_topic_name = "/can_twist";


  node->declare_parameter("twist_topic",subscribe_twist_topic_name);
  node->declare_parameter("yawrate_offset_stop.stop_judgment_velocity_threshold",yawrate_offset_stop_parameter.stop_judgment_velocity_threshold);
  node->declare_parameter("yawrate_offset_stop.estimated_number",yawrate_offset_stop_parameter.estimated_number);
  node->declare_parameter("yawrate_offset_stop.outlier_threshold",yawrate_offset_stop_parameter.outlier_threshold);

  node->get_parameter("twist_topic",subscribe_twist_topic_name);
  node->get_parameter("yawrate_offset_stop.stop_judgment_velocity_threshold",yawrate_offset_stop_parameter.stop_judgment_velocity_threshold);
  node->get_parameter("yawrate_offset_stop.estimated_number",yawrate_offset_stop_parameter.estimated_number);
  node->get_parameter("yawrate_offset_stop.outlier_threshold",yawrate_offset_stop_parameter.outlier_threshold);

  std::cout<< "subscribe_twist_topic_name "<<subscribe_twist_topic_name<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<yawrate_offset_stop_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "estimated_number "<<yawrate_offset_stop_parameter.estimated_number<<std::endl;
  std::cout<< "outlier_threshold "<<yawrate_offset_stop_parameter.outlier_threshold<<std::endl;

  auto sub1 = node->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, velocity_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  pub = node->create_publisher<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
