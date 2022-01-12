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
 * height_node.cpp
 * Author MapIV  Takanose
 */

 #include "rclcpp/rclcpp.hpp"
 #include "eagleye_coordinate/eagleye_coordinate.hpp"
 #include "eagleye_navigation/eagleye_navigation.hpp"

 static sensor_msgs::msg::Imu imu;
 static sensor_msgs::msg::NavSatFix fix;
 static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
 static eagleye_msgs::msg::Distance distance;

 rclcpp::Publisher<eagleye_msgs::msg::Height>::SharedPtr pub1;
 rclcpp::Publisher<eagleye_msgs::msg::Pitching>::SharedPtr pub2;
 rclcpp::Publisher<eagleye_msgs::msg::AccXOffset>::SharedPtr pub3;
 rclcpp::Publisher<eagleye_msgs::msg::AccXScaleFactor>::SharedPtr pub4;
 rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub5;
 static eagleye_msgs::msg::Height height;
 static eagleye_msgs::msg::Pitching pitching;
 static eagleye_msgs::msg::AccXOffset acc_x_offset;
 static eagleye_msgs::msg::AccXScaleFactor acc_x_scale_factor;

 struct HeightParameter height_parameter;
 struct HeightStatus height_status;

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  fix.header = msg->header;
  fix.status = msg->status;
  fix.latitude = msg->latitude;
  fix.longitude = msg->longitude;
  fix.altitude = msg->altitude;
  fix.position_covariance = msg->position_covariance;
  fix.position_covariance_type = msg->position_covariance_type;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
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
  height.header = msg->header;
  height.header.frame_id = "base_link";
  pitching.header = msg->header;
  pitching.header.frame_id = "base_link";
  acc_x_offset.header = msg->header;
  acc_x_scale_factor.header = msg->header;
  pitching_estimate(imu,fix,velocity_scale_factor,distance,height_parameter,&height_status,&height,&pitching,&acc_x_offset,&acc_x_scale_factor);
  pub1->publish(height);
  pub2->publish(pitching);
  pub3->publish(acc_x_offset);
  pub4->publish(acc_x_scale_factor);

  if(height_status.flag_reliability == true)
  {
    pub5->publish(fix);
  }

  height_status.flag_reliability = false;
  height.status.estimate_status = false;
  pitching.status.estimate_status = false;
  acc_x_offset.status.estimate_status = false;
  acc_x_scale_factor.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("height");

  std::string subscribe_navsatfix_topic_name = "/navsat/fix";
  std::string subscribe_imu_topic_name = "/imu/data_raw";

  node->declare_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->declare_parameter("imu_topic",subscribe_imu_topic_name);
  node->declare_parameter("height.estimated_distance",height_parameter.estimated_distance);
  node->declare_parameter("height.estimated_distance_max",height_parameter.estimated_distance_max);
  node->declare_parameter("height.separation_distance",height_parameter.separation_distance);
  node->declare_parameter("height.estimated_velocity_threshold",height_parameter.estimated_velocity_threshold);
  node->declare_parameter("height.estimated_velocity_coefficient",height_parameter.estimated_velocity_coefficient);
  node->declare_parameter("height.estimated_height_coefficient",height_parameter.estimated_height_coefficient);
  node->declare_parameter("height.outlier_threshold",height_parameter.outlier_threshold);
  node->declare_parameter("height.average_num",height_parameter.average_num);

  node->get_parameter("navsatfix_topic",subscribe_navsatfix_topic_name);
  node->get_parameter("imu_topic",subscribe_imu_topic_name);
  node->get_parameter("height.estimated_distance",height_parameter.estimated_distance);
  node->get_parameter("height.estimated_distance_max",height_parameter.estimated_distance_max);
  node->get_parameter("height.separation_distance",height_parameter.separation_distance);
  node->get_parameter("height.estimated_velocity_threshold",height_parameter.estimated_velocity_threshold);
  node->get_parameter("height.estimated_velocity_coefficient",height_parameter.estimated_velocity_coefficient);
  node->get_parameter("height.estimated_height_coefficient",height_parameter.estimated_height_coefficient);
  node->get_parameter("height.outlier_threshold",height_parameter.outlier_threshold);
  node->get_parameter("height.average_num",height_parameter.average_num);

  std::cout<< "subscribe_navsatfix_topic_name "<<subscribe_navsatfix_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "estimated_distance "<<height_parameter.estimated_distance<<std::endl;
  std::cout<< "estimated_distance_max "<<height_parameter.estimated_distance_max<<std::endl;
  std::cout<< "separation_distance "<<height_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<height_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_velocity_coefficient "<<height_parameter.estimated_velocity_coefficient<<std::endl;
  std::cout<< "estimated_height_coefficient "<<height_parameter.estimated_height_coefficient<<std::endl;
  std::cout<< "outlier_threshold "<<height_parameter.outlier_threshold<<std::endl;
  std::cout<< "average_num "<<height_parameter.average_num<<std::endl;

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>(subscribe_imu_topic_name, 1000, imu_callback); //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_navsatfix_topic_name, 1000, fix_callback); //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback); //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", rclcpp::QoS(10), distance_callback); //ros::TransportHints().tcpNoDelay()

  std::string publish_height_topic_name = "height";
  std::string publish_pitching_topic_name = "pitching";
  std::string publish_acc_x_offset_topic_name = "acc_x_offset";
  std::string publish_acc_x_scale_factor_topic_name = "acc_x_scale_factor";
  std::string publish_nav_sat_fix_topic_name = "navsat/reliability_fix";

  pub1 = node->create_publisher<eagleye_msgs::msg::Height>(publish_height_topic_name, 1000);
  pub2 = node->create_publisher<eagleye_msgs::msg::Pitching>(publish_pitching_topic_name, 1000);
  pub3 = node->create_publisher<eagleye_msgs::msg::AccXOffset>(publish_acc_x_offset_topic_name, 1000);
  pub4 = node->create_publisher<eagleye_msgs::msg::AccXScaleFactor>(publish_acc_x_scale_factor_topic_name, 1000);
  pub5 = node->create_publisher<sensor_msgs::msg::NavSatFix>(publish_nav_sat_fix_topic_name, 1000);

  rclcpp::spin(node);

  return 0;
}
