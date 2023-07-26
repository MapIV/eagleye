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
 * yaw_rate_offset.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static geometry_msgs::msg::TwistStamped _velocity;
static eagleye_msgs::msg::StatusStamped _velocity_status;
static eagleye_msgs::msg::YawrateOffset _yaw_rate_offset_stop;
static eagleye_msgs::msg::Heading _heading_interpolate;
static sensor_msgs::msg::Imu _imu;
rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr _pub;
static eagleye_msgs::msg::YawrateOffset _yaw_rate_offset;

struct YawrateOffsetParameter _yaw_rate_offset_parameter;
struct YawrateOffsetStatus _yaw_rate_offset_status;

bool _is_first_heading = false;
static bool _use_can_less_mode = false;

double _previous_yaw_rate_offset = 0.0;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  _velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstSharedPtr msg)
{
  _velocity_status = *msg;
}

void yaw_rate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  _yaw_rate_offset_stop = *msg;
}

void heading_interpolate_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  _heading_interpolate = *msg;
  if (_is_first_heading == false && _heading_interpolate.status.enabled_status == true)
  {
    _is_first_heading = true;
  }
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (_is_first_heading == false) return;
  if(_use_can_less_mode && !_velocity_status.status.enabled_status) return;

  _imu = *msg;
  _yaw_rate_offset.header = msg->header;
  yaw_rate_offset_estimate(_velocity,_yaw_rate_offset_stop,_heading_interpolate,_imu,_yaw_rate_offset_parameter, &_yaw_rate_offset_status, &_yaw_rate_offset);

  _yaw_rate_offset.status.is_abnormal = false;
  if (!std::isfinite(_yaw_rate_offset_stop.yaw_rate_offset)) {
    _yaw_rate_offset_stop.yaw_rate_offset =_previous_yaw_rate_offset;
    _yaw_rate_offset.status.is_abnormal = true;
    _yaw_rate_offset.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
  }
  else
  {
    _previous_yaw_rate_offset = _yaw_rate_offset_stop.yaw_rate_offset;
  }

  _pub->publish(_yaw_rate_offset);
  _yaw_rate_offset.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_yaw_rate_offset");

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "yaw_rate_offset_1st";
      subscribe_topic_name = "heading_interpolate_1st";

      try
      {
        YAML::Node conf = YAML::LoadFile(yaml_file);

        _yaw_rate_offset_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
        _yaw_rate_offset_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
        _yaw_rate_offset_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();

        _yaw_rate_offset_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["estimated_minimum_interval"].as<double>();
        _yaw_rate_offset_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["1st"]["estimated_maximum_interval"].as<double>();
        _yaw_rate_offset_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["gnss_receiving_threshold"].as<double>();
        _yaw_rate_offset_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

        std::cout << "imu_rate " << _yaw_rate_offset_parameter.imu_rate << std::endl;
        std::cout << "gnss_rate " << _yaw_rate_offset_parameter.gnss_rate << std::endl;
        std::cout << "moving_judgment_threshold " << _yaw_rate_offset_parameter.moving_judgment_threshold << std::endl;

        std::cout << "estimated_minimum_interval " << _yaw_rate_offset_parameter.estimated_minimum_interval << std::endl;
        std::cout << "estimated_maximum_interval " << _yaw_rate_offset_parameter.estimated_maximum_interval << std::endl;
        std::cout << "gnss_receiving_threshold " << _yaw_rate_offset_parameter.gnss_receiving_threshold << std::endl;
        std::cout << "outlier_threshold " << _yaw_rate_offset_parameter.outlier_threshold << std::endl;
      }
      catch (YAML::Exception& e)
      {
        std::cerr << "\033[1;yaw_rate_offset_1st Node YAML Error: " << e.msg << "\033[0m" << std::endl;
        exit(3);
      }
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "yaw_rate_offset_2nd";
      subscribe_topic_name = "heading_interpolate_2nd";
      
      try
      {
        YAML::Node conf = YAML::LoadFile(yaml_file);

        _yaw_rate_offset_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
        _yaw_rate_offset_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
        _yaw_rate_offset_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["moving_judgment_threshold"].as<double>();

        _yaw_rate_offset_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["estimated_minimum_interval"].as<double>();
        _yaw_rate_offset_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["2nd"]["estimated_maximum_interval"].as<double>();
        _yaw_rate_offset_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset"]["gnss_receiving_threshold"].as<double>();
        _yaw_rate_offset_parameter.outlier_threshold = conf["/**"]["ros__parameters"]["yaw_rate_offset_stop"]["outlier_threshold"].as<double>();

        std::cout << "imu_rate " << _yaw_rate_offset_parameter.imu_rate << std::endl;
        std::cout << "gnss_rate " << _yaw_rate_offset_parameter.gnss_rate << std::endl;
        std::cout << "moving_judgment_threshold " << _yaw_rate_offset_parameter.moving_judgment_threshold << std::endl;

        std::cout << "estimated_minimum_interval " << _yaw_rate_offset_parameter.estimated_minimum_interval << std::endl;
        std::cout << "estimated_maximum_interval " << _yaw_rate_offset_parameter.estimated_maximum_interval << std::endl;
        std::cout << "gnss_receiving_threshold " << _yaw_rate_offset_parameter.gnss_receiving_threshold << std::endl;
        std::cout << "outlier_threshold " << _yaw_rate_offset_parameter.outlier_threshold << std::endl;
      }
      catch (YAML::Exception& e)
      {
        std::cerr << "\033[1;yaw_rate_offset_2nd Node YAML Error: " << e.msg << "\033[0m" << std::endl;
        exit(3);
      }
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

  auto sub1 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yaw_rate_offset_stop", rclcpp::QoS(10), yaw_rate_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub3 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name, 1000, heading_interpolate_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub4 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  _pub = node->create_publisher<eagleye_msgs::msg::YawrateOffset>(publish_topic_name, rclcpp::QoS(10));

  rclcpp::spin(node);


  return 0;
}
