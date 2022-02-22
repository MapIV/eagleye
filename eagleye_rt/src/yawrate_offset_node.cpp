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
 * yawrate_offset.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::msg::Heading heading_interpolate;
static sensor_msgs::msg::Imu imu;
rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr pub;
static eagleye_msgs::msg::YawrateOffset yawrate_offset;

struct YawrateOffsetParameter yawrate_offset_parameter;
struct YawrateOffsetStatus yawrate_offset_status;

bool is_first_heading= false;

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate = *msg;
  if (is_first_heading == false && heading_interpolate.status.enabled_status == true)
  {
    is_first_heading = true;
  }
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (is_first_heading == false)
  {
    return;
  }
  imu = *msg;
  yawrate_offset.header = msg->header;
  yawrate_offset_estimate(velocity_scale_factor,yawrate_offset_stop,heading_interpolate,imu, yawrate_offset_parameter, &yawrate_offset_status, &yawrate_offset);
  pub->publish(yawrate_offset);
  yawrate_offset.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yawrate_offset");

  std::string subscribe_imu_topic_name = "/imu/data_raw";

  node->declare_parameter("imu_topic",subscribe_imu_topic_name);
  node->declare_parameter("reverse_imu", yawrate_offset_parameter.reverse_imu);
  node->declare_parameter("yawrate_offset.estimated_number_min",yawrate_offset_parameter.estimated_number_min);
  node->declare_parameter("yawrate_offset.estimated_coefficient",yawrate_offset_parameter.estimated_coefficient);
  node->declare_parameter("yawrate_offset.estimated_velocity_threshold",yawrate_offset_parameter.estimated_velocity_threshold);
  node->declare_parameter("yawrate_offset.outlier_threshold",yawrate_offset_parameter.outlier_threshold);

  node->get_parameter("imu_topic",subscribe_imu_topic_name);
  node->get_parameter("reverse_imu", yawrate_offset_parameter.reverse_imu);
  node->get_parameter("yawrate_offset.estimated_number_min",yawrate_offset_parameter.estimated_number_min);
  node->get_parameter("yawrate_offset.estimated_coefficient",yawrate_offset_parameter.estimated_coefficient);
  node->get_parameter("yawrate_offset.estimated_velocity_threshold",yawrate_offset_parameter.estimated_velocity_threshold);
  node->get_parameter("yawrate_offset.outlier_threshold",yawrate_offset_parameter.outlier_threshold);

  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<yawrate_offset_parameter.reverse_imu<<std::endl;
  std::cout<< "estimated_number_min "<<yawrate_offset_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_coefficient "<<yawrate_offset_parameter.estimated_coefficient<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<yawrate_offset_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<yawrate_offset_parameter.outlier_threshold<<std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "yawrate_offset_1st";
      subscribe_topic_name = "heading_interpolate_1st";
      node->declare_parameter("yawrate_offset.1st.estimated_number_max",yawrate_offset_parameter.estimated_number_max);
      node->get_parameter("yawrate_offset.1st.estimated_number_max",yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max "<<yawrate_offset_parameter.estimated_number_max<<std::endl;
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "yawrate_offset_2nd";
      subscribe_topic_name = "heading_interpolate_2nd";
      node->declare_parameter("yawrate_offset.2nd.estimated_number_max",yawrate_offset_parameter.estimated_number_max);
      node->get_parameter("yawrate_offset.2nd.estimated_number_max",yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max "<<yawrate_offset_parameter.estimated_number_max<<std::endl;
    }
    else
    {
      // RCLCPP_ERROR(node->get_logger(),"Invalid argument");
      RCLCPP_ERROR(node->get_logger(), "No arguments");
      rclcpp::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "No arguments");
    rclcpp::shutdown();
  }

  auto sub1 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name, 1000, heading_interpolate_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  pub = node->create_publisher<eagleye_msgs::msg::YawrateOffset>(publish_topic_name, rclcpp::QoS(10));

  rclcpp::spin(node);


  return 0;
}
