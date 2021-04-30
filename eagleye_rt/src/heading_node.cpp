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
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

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

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void yawrate_offset_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset.header = msg->header;
  yawrate_offset.yawrate_offset = msg->yawrate_offset;
  yawrate_offset.status = msg->status;
}

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle.header = msg->header;
  slip_angle.coefficient = msg->coefficient;
  slip_angle.slip_angle = msg->slip_angle;
  slip_angle.status = msg->status;
}

void heading_interpolate_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate.header = msg->header;
  heading_interpolate.heading_angle = msg->heading_angle;
  heading_interpolate.status = msg->status;
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
  heading.header = msg->header;
  heading.header.frame_id = "enu";
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

  // n.getParam("eagleye/imu_topic",subscribe_imu_topic_name);
  // n.getParam("eagleye/rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  // n.getParam("eagleye/reverse_imu", heading_parameter.reverse_imu);
  // n.getParam("eagleye/heading/estimated_number_min",heading_parameter.estimated_number_min);
  // n.getParam("eagleye/heading/estimated_number_max",heading_parameter.estimated_number_max);
  // n.getParam("eagleye/heading/estimated_gnss_coefficient",heading_parameter.estimated_gnss_coefficient);
  // n.getParam("eagleye/heading/estimated_heading_coefficient",heading_parameter.estimated_heading_coefficient);
  // n.getParam("eagleye/heading/outlier_threshold",heading_parameter.outlier_threshold);
  // n.getParam("eagleye/heading/estimated_velocity_threshold",heading_parameter.estimated_velocity_threshold);
  // n.getParam("eagleye/heading/stop_judgment_velocity_threshold",heading_parameter.stop_judgment_velocity_threshold);
  // n.getParam("eagleye/heading/estimated_yawrate_threshold",heading_parameter.estimated_yawrate_threshold);

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

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "eagleye/heading_1st";
      subscribe_topic_name = "eagleye/yawrate_offset_stop";
      subscribe_topic_name2 = "eagleye/heading_interpolate_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "eagleye/heading_2nd";
      subscribe_topic_name = "eagleye/yawrate_offset_1st";
      subscribe_topic_name2 = "eagleye/heading_interpolate_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "eagleye/heading_3rd";
      subscribe_topic_name = "eagleye/yawrate_offset_2nd";
      subscribe_topic_name2 = "eagleye/heading_interpolate_3rd";
    }
    else
    {
      // ROS_ERROR("Invalid argument");
      RCLCPP_ERROR(node->get_logger(),"Invalid argument");
      rclcpp::shutdown();
    }
  }
  else
  {
    // ROS_ERROR("No arguments");
    RCLCPP_ERROR(node->get_logger(),"No arguments");
    rclcpp::shutdown();
  }

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>(subscribe_topic_name, 1000, yawrate_offset_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub6 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("eagleye/slip_angle", 1000, slip_angle_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub7 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name2 , 1000, heading_interpolate_callback);  //ros::TransportHints().tcpNoDelay()
  pub = node->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, 1000);

  rclcpp::spin(node);

  return 0;
}
