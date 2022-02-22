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
 * heading.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static sensor_msgs::msg::Imu imu;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yawrate_offset;
static eagleye_msgs::msg::SlipAngle slip_angle;
static eagleye_msgs::msg::Heading heading_interpolate;

rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub;
static eagleye_msgs::msg::Heading heading;

struct HeadingParameter heading_parameter;
struct HeadingStatus heading_status;

bool is_first_correction_velocity = false;

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor = *msg;
  if (is_first_correction_velocity == false && msg->correction_velocity.linear.x > heading_parameter.estimated_velocity_threshold)
  {
    is_first_correction_velocity = true;
  }
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop = *msg;
}

void yawrate_offset_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset = *msg;
}

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (is_first_correction_velocity == false)
  {
    return;
  }
  imu = *msg;
  heading.header = msg->header;
  heading.header.frame_id = "base_link";
  heading_estimate(rtklib_nav,imu,velocity_scale_factor,yawrate_offset_stop,yawrate_offset,slip_angle,heading_interpolate,heading_parameter,&heading_status,&heading);

  if (heading.status.estimate_status == true)
  {
    pub->publish(heading);
  }
  heading.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("heading");

  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  node->declare_parameter("imu_topic",subscribe_imu_topic_name);
  node->declare_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->declare_parameter("reverse_imu", heading_parameter.reverse_imu);
  node->declare_parameter("heading.estimated_number_min",heading_parameter.estimated_number_min);
  node->declare_parameter("heading.estimated_number_max",heading_parameter.estimated_number_max);
  node->declare_parameter("heading.estimated_gnss_coefficient",heading_parameter.estimated_gnss_coefficient);
  node->declare_parameter("heading.estimated_heading_coefficient",heading_parameter.estimated_heading_coefficient);
  node->declare_parameter("heading.outlier_threshold",heading_parameter.outlier_threshold);
  node->declare_parameter("heading.estimated_velocity_threshold",heading_parameter.estimated_velocity_threshold);
  node->declare_parameter("heading.stop_judgment_velocity_threshold",heading_parameter.stop_judgment_velocity_threshold);
  node->declare_parameter("heading.estimated_yawrate_threshold",heading_parameter.estimated_yawrate_threshold);

  node->get_parameter("imu_topic",subscribe_imu_topic_name);
  node->get_parameter("rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  node->get_parameter("reverse_imu", heading_parameter.reverse_imu);
  node->get_parameter("heading.estimated_number_min",heading_parameter.estimated_number_min);
  node->get_parameter("heading.estimated_number_max",heading_parameter.estimated_number_max);
  node->get_parameter("heading.estimated_gnss_coefficient",heading_parameter.estimated_gnss_coefficient);
  node->get_parameter("heading.estimated_heading_coefficient",heading_parameter.estimated_heading_coefficient);
  node->get_parameter("heading.outlier_threshold",heading_parameter.outlier_threshold);
  node->get_parameter("heading.estimated_velocity_threshold",heading_parameter.estimated_velocity_threshold);
  node->get_parameter("heading.stop_judgment_velocity_threshold",heading_parameter.stop_judgment_velocity_threshold);
  node->get_parameter("heading.estimated_yawrate_threshold",heading_parameter.estimated_yawrate_threshold);

  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<heading_parameter.reverse_imu<<std::endl;
  std::cout<< "estimated_number_min "<<heading_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<heading_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_gnss_coefficient "<<heading_parameter.estimated_gnss_coefficient<<std::endl;
  std::cout<< "estimated_heading_coefficient "<<heading_parameter.estimated_heading_coefficient<<std::endl;
  std::cout<< "outlier_threshold "<<heading_parameter.outlier_threshold<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<heading_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<heading_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "estimated_yawrate_threshold "<<heading_parameter.estimated_yawrate_threshold<<std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";
  std::string subscribe_topic_name2 = "/subscribe_topic_name2/invalid";

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "heading_1st";
      subscribe_topic_name = "yawrate_offset_stop";
      subscribe_topic_name2 = "heading_interpolate_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "heading_2nd";
      subscribe_topic_name = "yawrate_offset_1st";
      subscribe_topic_name2 = "heading_interpolate_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "heading_3rd";
      subscribe_topic_name = "yawrate_offset_2nd";
      subscribe_topic_name2 = "heading_interpolate_3rd";
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(),"Invalid argument");
      rclcpp::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(),"No arguments");
    rclcpp::shutdown();
  }

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>(subscribe_topic_name, 1000, yawrate_offset_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub6 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10), slip_angle_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub7 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name2 , 1000, heading_interpolate_callback);  //ros::TransportHints().tcpNoDelay()
  pub = node->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, rclcpp::QoS(10));

  rclcpp::spin(node);


  return 0;
}
