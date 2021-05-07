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

#include "rclcpp/rclcpp.hpp"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static sensor_msgs::msg::Imu imu;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::msg::Pitching pitching;


static geometry_msgs::msg::Vector3Stamped enu_vel;
static eagleye_msgs::msg::Position enu_relative_pos;
static geometry_msgs::msg::TwistStamped eagleye_twist;
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub1;
rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub2;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub3;

struct TrajectoryParameter trajectory_parameter;
struct TrajectoryStatus trajectory_status;

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
{
  pitching.header = msg->header;
  pitching.pitching_angle = msg->pitching_angle;
  pitching.status = msg->status;
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
  enu_vel.header = msg->header;
  enu_vel.header.frame_id = "gps";
  enu_relative_pos.header = msg->header;
  enu_relative_pos.header.frame_id = "enu";
  eagleye_twist.header = msg->header;
  eagleye_twist.header.frame_id = "base_link";
  trajectory3d_estimate(imu,velocity_scale_factor,heading_interpolate_3rd,yawrate_offset_stop,yawrate_offset_2nd,pitching,trajectory_parameter,&trajectory_status,&enu_vel,&enu_relative_pos,&eagleye_twist);

  if(heading_interpolate_3rd.status.enabled_status == true)
  {
    pub1->publish(enu_vel);
    pub2->publish(enu_relative_pos);
    pub3->publish(eagleye_twist);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trajectory");

  std::string subscribe_imu_topic_name = "/imu/data_raw";

  node->declare_parameter("imu_topic",subscribe_imu_topic_name);
  node->declare_parameter("reverse_imu", trajectory_parameter.reverse_imu);
  node->declare_parameter("trajectory.stop_judgment_velocity_threshold",trajectory_parameter.stop_judgment_velocity_threshold);
  node->declare_parameter("trajectory.stop_judgment_yawrate_threshold",trajectory_parameter.stop_judgment_yawrate_threshold);

  node->get_parameter("imu_topic",subscribe_imu_topic_name);
  node->get_parameter("reverse_imu", trajectory_parameter.reverse_imu);
  node->get_parameter("trajectory.stop_judgment_velocity_threshold",trajectory_parameter.stop_judgment_velocity_threshold);
  node->get_parameter("trajectory.stop_judgment_yawrate_threshold",trajectory_parameter.stop_judgment_yawrate_threshold);
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<trajectory_parameter.reverse_imu<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<trajectory_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "stop_judgment_yawrate_threshold "<<trajectory_parameter.stop_judgment_yawrate_threshold<<std::endl;

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("eagleye/velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::Heading>("eagleye/heading_interpolate_3rd", rclcpp::QoS(10), heading_interpolate_3rd_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("eagleye/yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("eagleye/yawrate_offset_2nd", rclcpp::QoS(10), yawrate_offset_2nd_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub6 = node->create_subscription<eagleye_msgs::msg::Pitching>("eagleye/pitching", rclcpp::QoS(10), pitching_callback);  //ros::TransportHints().tcpNoDelay()
  pub1 = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("eagleye/enu_vel", 1000);
  pub2 = node->create_publisher<eagleye_msgs::msg::Position>("eagleye/enu_relative_pos", 1000);
  pub3 = node->create_publisher<geometry_msgs::msg::TwistStamped>("eagleye/twist", 1000);

  rclcpp::spin(node);

  return 0;
}
