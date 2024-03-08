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
static nmea_msgs::msg::Gprmc nmea_rmc;
static eagleye_msgs::msg::Heading multi_antenna_heading;
static sensor_msgs::msg::Imu imu;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yaw_rate_offset;
static eagleye_msgs::msg::SlipAngle slip_angle;
static eagleye_msgs::msg::Heading heading_interpolate;

rclcpp::Publisher<eagleye_msgs::msg::Heading>::SharedPtr pub;
static eagleye_msgs::msg::Heading heading;

struct HeadingParameter heading_parameter;
struct HeadingStatus heading_status;

std::string use_gnss_mode;
static bool use_can_less_mode;
static bool use_multi_antenna_mode;

bool is_first_correction_velocity = false;

std::string node_name = "eagleye_heading";

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;

  // To avoid unnecessary buffering when it's just sitting there right after start-up, we're making it so it doesn't buffer.
  // Multi-antenna mode is an exception.
  if (is_first_correction_velocity == false && msg->twist.linear.x > heading_parameter.moving_judgment_threshold)
  {
    is_first_correction_velocity = true;
  }
}

void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  tf2::Quaternion orientation;
  tf2::fromMsg(msg->pose.orientation, orientation);
  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  double heading = - yaw + (90* M_PI / 180);

  multi_antenna_heading.header = msg->header;
  multi_antenna_heading.heading_angle = heading;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  velocity_status = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset_stop = *msg;
}

void yaw_rate_offset_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yaw_rate_offset = *msg;
}

void slip_angle_callback(const eagleye_msgs::msg::SlipAngle::ConstSharedPtr msg)
{
  slip_angle = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate = *msg;
}

void rmc_callback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg)
{
  nmea_rmc = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!is_first_correction_velocity)
  {
    RCLCPP_WARN(rclcpp::get_logger(node_name), "is_first_correction_velocity is false.");
    return;
  }
  if(use_can_less_mode && !velocity_status.status.enabled_status)
  {
    RCLCPP_WARN(rclcpp::get_logger(node_name), "velocity_status is not enabled.");
    return;
  }
  if(!yaw_rate_offset_stop.status.enabled_status)
  {
    RCLCPP_WARN(rclcpp::get_logger(node_name), "Heading estimation is not started because the stop calibration is not yet completed.");
    return;
  }

  imu = *msg;
  heading.header = msg->header;
  heading.header.frame_id = "base_link";
  bool use_rtklib_mode = use_gnss_mode == "rtklib" || use_gnss_mode == "RTKLIB";
  bool use_nmea_mode = use_gnss_mode == "nmea" || use_gnss_mode == "NMEA";
  if (use_rtklib_mode && !use_multi_antenna_mode) // use RTKLIB mode
    heading_estimate(rtklib_nav,imu,velocity,yaw_rate_offset_stop,yaw_rate_offset,slip_angle,heading_interpolate,heading_parameter,&heading_status,&heading);
  else if (use_nmea_mode && !use_multi_antenna_mode) // use NMEA mode
    heading_estimate(nmea_rmc,imu,velocity,yaw_rate_offset_stop,yaw_rate_offset,slip_angle,heading_interpolate,heading_parameter,&heading_status,&heading);
  else if (use_multi_antenna_mode)
    heading_estimate(multi_antenna_heading, imu, velocity, yaw_rate_offset_stop, yaw_rate_offset, slip_angle,
      heading_interpolate, heading_parameter, &heading_status, &heading);

  if (heading.status.estimate_status == true || use_multi_antenna_mode)
  {
    pub->publish(heading);
  }
  heading.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(node_name);

  std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
  std::string subscribe_rmc_topic_name = "gnss/rmc";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  node->declare_parameter("use_multi_antenna_mode",use_multi_antenna_mode);
  node->get_parameter("use_multi_antenna_mode",use_multi_antenna_mode);
  std::cout << "yaml_file: " << yaml_file << std::endl;
  std::cout << "use_multi_antenna_mode: " << use_multi_antenna_mode << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    use_gnss_mode = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();

    heading_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    heading_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    heading_parameter.stop_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();
    heading_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();
    heading_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["heading"]["estimated_minimum_interval"].as<double>();
    heading_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["heading"]["estimated_maximum_interval"].as<double>();
    heading_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["heading"]["gnss_receiving_threshold"].as<double>();
    heading_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["heading"]["outlier_threshold"].as<double>();
    heading_parameter.outlier_ratio_threshold = conf["/**"]["ros__parameters"]["heading"]["outlier_ratio_threshold"].as<double>();
    heading_parameter.curve_judgment_threshold = conf["/**"]["ros__parameters"]["heading"]["curve_judgment_threshold"].as<double>();
    heading_parameter.init_STD = conf["/**"]["ros__parameters"]["heading"]["init_STD"].as<double>();

    std::cout<< "use_gnss_mode " << use_gnss_mode << std::endl;

    std::cout<< "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;
    std::cout<< "subscribe_rmc_topic_name " << subscribe_rmc_topic_name << std::endl;

    std::cout << "imu_rate " << heading_parameter.imu_rate << std::endl;
    std::cout << "gnss_rate " << heading_parameter.gnss_rate << std::endl;
    std::cout << "stop_judgment_threshold " << heading_parameter.stop_judgment_threshold << std::endl;
    std::cout << "moving_judgment_threshold " << heading_parameter.moving_judgment_threshold << std::endl;

    std::cout << "estimated_minimum_interval " << heading_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << heading_parameter.estimated_maximum_interval << std::endl;
    std::cout << "gnss_receiving_threshold " << heading_parameter.gnss_receiving_threshold << std::endl;
    std::cout << "outlier_threshold " << heading_parameter.outlier_threshold << std::endl;
    std::cout << "outlier_ratio_threshold " << heading_parameter.outlier_ratio_threshold << std::endl;
    std::cout << "curve_judgment_threshold " << heading_parameter.curve_judgment_threshold << std::endl;
    std::cout << "init_STD " << heading_parameter.init_STD << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mheading Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";
  std::string subscribe_topic_name2 = "/subscribe_topic_name2/invalid";

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "heading_1st";
      subscribe_topic_name = "yaw_rate_offset_stop";
      subscribe_topic_name2 = "heading_interpolate_1st";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "heading_2nd";
      subscribe_topic_name = "yaw_rate_offset_1st";
      subscribe_topic_name2 = "heading_interpolate_2nd";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "heading_3rd";
      subscribe_topic_name = "yaw_rate_offset_2nd";
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

  if(use_multi_antenna_mode)
  {
    is_first_correction_velocity = true;
  }

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto sub2 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub3 = node->create_subscription<nmea_msgs::msg::Gprmc>(subscribe_rmc_topic_name, 1000, rmc_callback);
  auto sub4 = node->create_subscription<geometry_msgs::msg::PoseStamped>("gnss_compass_pose", 1000, pose_callback);
  auto sub5 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub6 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub7 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), yaw_rate_offset_stop_callback);
  auto sub8 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>(subscribe_topic_name, 1000, yaw_rate_offset_callback);
  auto sub9 = node->create_subscription<eagleye_msgs::msg::SlipAngle>("slip_angle", rclcpp::QoS(10), slip_angle_callback);
  auto sub10 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name2 , 1000, heading_interpolate_callback);
  pub = node->create_publisher<eagleye_msgs::msg::Heading>(publish_topic_name, rclcpp::QoS(10));

  rclcpp::spin(node);


  return 0;
}
