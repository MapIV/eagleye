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
 * slip_coefficient.cpp
 * Author MapIV Takanose
 */

#include "rclcpp/rclcpp.hpp"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static rtklib_msgs::msg::RtklibNav rtklib_nav;
static sensor_msgs::msg::Imu imu;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;

struct SlipCoefficientParameter slip_coefficient_parameter;
struct SlipCoefficientStatus slip_coefficient_status;

static double estimate_coefficient;

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

void yawrate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
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
  slip_coefficient_estimate(imu,rtklib_nav,velocity_scale_factor,yawrate_offset_stop,yawrate_offset_2nd,heading_interpolate_3rd,slip_coefficient_parameter,&slip_coefficient_status,&estimate_coefficient);

  std::cout << "--- \033[1;34m slip_coefficient \033[m ------------------------------"<< std::endl;
  std::cout<<"\033[1m estimate_coefficient \033[mx "<<estimate_coefficient<<std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("slip_coefficient");


  std::string subscribe_imu_topic_name = "/imu/data_raw";
  std::string subscribe_rtklib_nav_topic_name = "/rtklib_nav";

  n.getParam("eagleye/imu_topic",subscribe_imu_topic_name);
  n.getParam("eagleye/rtklib_nav_topic",subscribe_rtklib_nav_topic_name);
  n.getParam("eagleye/reverse_imu", slip_coefficient_parameter.reverse_imu);
  n.getParam("eagleye/slip_coefficient/estimated_number_min", slip_coefficient_parameter.estimated_number_min);
  n.getParam("eagleye/slip_coefficient/estimated_number_max", slip_coefficient_parameter.estimated_number_max);
  n.getParam("eagleye/slip_coefficient/estimated_velocity_threshold", slip_coefficient_parameter.estimated_velocity_threshold);
  n.getParam("eagleye/slip_coefficient/estimated_yawrate_threshold", slip_coefficient_parameter.estimated_yawrate_threshold);
  n.getParam("eagleye/slip_coefficient/lever_arm", slip_coefficient_parameter.lever_arm);
  n.getParam("eagleye/slip_coefficient/stop_judgment_velocity_threshold", slip_coefficient_parameter.stop_judgment_velocity_threshold);

  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "subscribe_rtklib_nav_topic_name "<<subscribe_rtklib_nav_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<slip_coefficient_parameter.reverse_imu<<std::endl;
  std::cout<< "estimated_number_min "<<slip_coefficient_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<slip_coefficient_parameter.estimated_number_max<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<slip_coefficient_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_yawrate_threshold "<<slip_coefficient_parameter.estimated_yawrate_threshold<<std::endl;
  std::cout<< "lever_arm "<<slip_coefficient_parameter.lever_arm<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<slip_coefficient_parameter.stop_judgment_velocity_threshold<<std::endl;

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback);
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub3 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("eagleye/yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::Heading>("eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);


  rclcpp::spin(node);

  FILE *fp;
  std::string str;
  node->declare_parameter(str, "output_dir");
  str = node->get_parameter(str).as_string();
  fp = fopen(str.c_str(),"w");
  fprintf(fp,"slip_coefficient %lf",estimate_coefficient);
  fclose(fp);

  return 0;
}
