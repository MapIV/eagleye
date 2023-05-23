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
 static nmea_msgs::msg::Gpgga gga;
 static geometry_msgs::msg::TwistStamped velocity;
 static eagleye_msgs::msg::StatusStamped velocity_status;
 static eagleye_msgs::msg::Distance distance;

 rclcpp::Publisher<eagleye_msgs::msg::Height>::SharedPtr pub1;
 rclcpp::Publisher<eagleye_msgs::msg::Pitching>::SharedPtr pub2;
 rclcpp::Publisher<eagleye_msgs::msg::AccXOffset>::SharedPtr pub3;
 rclcpp::Publisher<eagleye_msgs::msg::AccXScaleFactor>::SharedPtr pub4;
 rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr pub5;
 static eagleye_msgs::msg::Height height;
 static eagleye_msgs::msg::Pitching pitching;
 static eagleye_msgs::msg::AccXOffset acc_x_offset;
 static eagleye_msgs::msg::AccXScaleFactor acc_x_scale_factor;

 struct HeightParameter height_parameter;
 struct HeightStatus height_status;

 static bool use_can_less_mode;

void gga_callback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
{
  gga = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status = *msg;
}

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  distance = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(use_can_less_mode && !velocity_status.status.enabled_status) return;

  imu = *msg;
  height.header = msg->header;
  height.header.frame_id = "base_link";
  pitching.header = msg->header;
  pitching.header.frame_id = "base_link";
  acc_x_offset.header = msg->header;
  acc_x_scale_factor.header = msg->header;
  pitching_estimate(imu,gga,velocity,distance,height_parameter,&height_status,&height,&pitching,&acc_x_offset,&acc_x_scale_factor);
  pub1->publish(height);
  pub2->publish(pitching);
  pub3->publish(acc_x_offset);
  pub4->publish(acc_x_scale_factor);

  if (height_status.flag_reliability == true)
  {
    pub5->publish(gga);
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
  auto node = rclcpp::Node::make_shared("eagleye_height");

  std::string subscribe_gga_topic_name = "gnss/gga";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    height_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    height_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    height_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();

    height_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["height"]["estimated_minimum_interval"].as<double>();
    height_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["height"]["estimated_maximum_interval"].as<double>();
    height_parameter.update_distance = conf["/**"]["ros__parameters"]["height"]["update_distance"].as<double>();
    height_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["height"]["gnss_receiving_threshold"].as<double>();
    height_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["height"]["outlier_threshold"].as<double>();
    height_parameter.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["height"]["outlier_ratio_threshold"].as<double>();
    height_parameter.moving_average_time = conf["/**"]["ros__parameters"]["height"]["moving_average_time"].as<double>();

    std::cout << "imu_rate " << height_parameter.imu_rate << std::endl;
    std::cout << "gnss_rate " << height_parameter.gnss_rate << std::endl;
    std::cout << "moving_judgment_threshold " << height_parameter.moving_judgment_threshold << std::endl;

    std::cout << "estimated_minimum_interval " << height_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << height_parameter.estimated_maximum_interval << std::endl;
    std::cout << "update_distance " << height_parameter.update_distance << std::endl;
    std::cout << "gnss_receiving_threshold " << height_parameter.gnss_receiving_threshold << std::endl;
    std::cout << "outlier_threshold " << height_parameter.outlier_threshold << std::endl;
    std::cout << "outlier_ratio_threshold " << height_parameter.outlier_ratio_threshold << std::endl;
    std::cout << "moving_average_time " << height_parameter.moving_average_time << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mheight Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }


  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto sub2 = node->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, gga_callback);
  auto sub3 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Distance>("distance", rclcpp::QoS(10), distance_callback);

  std::string publish_height_topic_name = "height";
  std::string publish_pitching_topic_name = "pitching";
  std::string publish_acc_x_offset_topic_name = "acc_x_offset";
  std::string publish_acc_x_scale_factor_topic_name = "acc_x_scale_factor";
  std::string publish_nav_sat_gga_topic_name = "navsat/reliability_gga";

  pub1 = node->create_publisher<eagleye_msgs::msg::Height>(publish_height_topic_name, 1000);
  pub2 = node->create_publisher<eagleye_msgs::msg::Pitching>(publish_pitching_topic_name, 1000);
  pub3 = node->create_publisher<eagleye_msgs::msg::AccXOffset>(publish_acc_x_offset_topic_name, 1000);
  pub4 = node->create_publisher<eagleye_msgs::msg::AccXScaleFactor>(publish_acc_x_scale_factor_topic_name, 1000);
  pub5 = node->create_publisher<nmea_msgs::msg::Gpgga>(publish_nav_sat_gga_topic_name, 1000);

  rclcpp::spin(node);

  return 0;
}
