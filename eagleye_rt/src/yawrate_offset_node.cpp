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

static geometry_msgs::msg::TwistStamped _velocity;
static eagleye_msgs::msg::StatusStamped _velocity_status;
static eagleye_msgs::msg::YawrateOffset _yawrate_offset_stop;
static eagleye_msgs::msg::Heading _heading_interpolate;
static sensor_msgs::msg::Imu _imu;
rclcpp::Publisher<eagleye_msgs::msg::YawrateOffset>::SharedPtr _pub;
static eagleye_msgs::msg::YawrateOffset _yawrate_offset;
static geometry_msgs::msg::TwistStamped::ConstSharedPtr _comparison_velocity_ptr;

struct YawrateOffsetParameter _yawrate_offset_parameter;
struct YawrateOffsetStatus _yawrate_offset_status;

bool _is_first_heading = false;
static bool _use_canless_mode = false;

double _previous_yawrate_offset = 0.0;

bool _use_compare_yawrate = false;
double _th_diff_rad_per_sec = 0.17453;
int _num_continuous_abnormal_yawrate = 0;
int _th_num_continuous_abnormal_yawrate = 10;

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  _velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstPtr msg)
{
  _velocity_status = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  _yawrate_offset_stop = *msg;
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
  if(_use_canless_mode && !_velocity_status.status.enabled_status) return;

  _imu = *msg;
  _yawrate_offset.header = msg->header;
  yawrate_offset_estimate(_velocity,_yawrate_offset_stop,_heading_interpolate,_imu,_yawrate_offset_parameter, &_yawrate_offset_status, &_yawrate_offset);

  _yawrate_offset.status.is_abnormal = false;
  if (!std::isfinite(_yawrate_offset.yawrate_offset))
  {
    _yawrate_offset.yawrate_offset =_previous_yawrate_offset;
    _yawrate_offset.status.is_abnormal = true;
    _yawrate_offset.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
  }
  else if(_use_compare_yawrate || _comparison_velocity_ptr != nullptr)
  {
    double corrected_angular_velocity_z =  -1 * (_imu.angular_velocity.z + _yawrate_offset.yawrate_offset);
          std::cout << "corrected_angular_velocity_z" << corrected_angular_velocity_z << std::endl;
      std::cout << "_comparison_velocity_ptr->twist.angular.z" << _comparison_velocity_ptr->twist.angular.z << std::endl;
    if(_th_diff_rad_per_sec <
    std::abs(corrected_angular_velocity_z - _comparison_velocity_ptr->twist.angular.z))
    {
      _num_continuous_abnormal_yawrate++;
    }
    else
    {
      _num_continuous_abnormal_yawrate = 0;
    }
    if (_num_continuous_abnormal_yawrate > _th_num_continuous_abnormal_yawrate) {
      _yawrate_offset.status.is_abnormal = true;
      _yawrate_offset.status.error_code = eagleye_msgs::msg::Status::TOO_LARGE_OR_SMALL;
    }
  }
  else
  {
    _previous_yawrate_offset = _yawrate_offset_stop.yawrate_offset;
  }

  _pub->publish(_yawrate_offset);
  _yawrate_offset.status.estimate_status = false;
}

void comparison_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  if(_comparison_velocity_ptr == nullptr)
  {
    RCLCPP_WARN(rclcpp::get_logger("yawrate_offset"), "The comparison twist is not subscribed.");
  }
  _comparison_velocity_ptr = msg;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yawrate_offset");

  std::string comparison_twist_topic_name = "/calculated_twist";

  node->declare_parameter("yawrate_offset.estimated_number_min",_yawrate_offset_parameter.estimated_number_min);
  node->declare_parameter("yawrate_offset.estimated_coefficient",_yawrate_offset_parameter.estimated_coefficient);
  node->declare_parameter("yawrate_offset.estimated_velocity_threshold",_yawrate_offset_parameter.estimated_velocity_threshold);
  node->declare_parameter("yawrate_offset.outlier_threshold",_yawrate_offset_parameter.outlier_threshold);
  node->declare_parameter("use_canless_mode",_use_canless_mode);
  node->declare_parameter("yawrate_offset.use_compare_yawrate",_use_compare_yawrate);
  node->declare_parameter("yawrate_offset.th_diff_rad_per_sec",_th_diff_rad_per_sec);
  node->declare_parameter("yawrate_offset.th_num_continuous_abnormal_yawrate",_th_num_continuous_abnormal_yawrate);

  node->get_parameter("yawrate_offset.estimated_number_min",_yawrate_offset_parameter.estimated_number_min);
  node->get_parameter("yawrate_offset.estimated_coefficient",_yawrate_offset_parameter.estimated_coefficient);
  node->get_parameter("yawrate_offset.estimated_velocity_threshold",_yawrate_offset_parameter.estimated_velocity_threshold);
  node->get_parameter("yawrate_offset.outlier_threshold",_yawrate_offset_parameter.outlier_threshold);
  node->get_parameter("use_canless_mode",_use_canless_mode);
  node->get_parameter("yawrate_offset.use_compare_yawrate",_use_compare_yawrate);
  node->get_parameter("yawrate_offset.th_diff_rad_per_sec",_th_diff_rad_per_sec);
  node->get_parameter("yawrate_offset.th_num_continuous_abnormal_yawrate",_th_num_continuous_abnormal_yawrate);

  std::cout<< "estimated_number_min "<<_yawrate_offset_parameter.estimated_number_min<<std::endl;
  std::cout<< "estimated_coefficient "<<_yawrate_offset_parameter.estimated_coefficient<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<_yawrate_offset_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "outlier_threshold "<<_yawrate_offset_parameter.outlier_threshold<<std::endl;
  std::cout<< "use_canless_mode "<<_use_canless_mode<<std::endl;
  std::cout<< "use_compare_yawrate "<<_use_compare_yawrate<<std::endl;
  if(_use_compare_yawrate) {
  std::cout<< "comparison_twist_topic_name "<<comparison_twist_topic_name<<std::endl;
  std::cout<< "th_diff_rad_per_sec "<<_th_diff_rad_per_sec<<std::endl;
  std::cout<< "th_num_continuous_abnormal_yawrate "<<_th_num_continuous_abnormal_yawrate<<std::endl;
  }

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc > 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "yawrate_offset_1st";
      subscribe_topic_name = "heading_interpolate_1st";
      node->declare_parameter("yawrate_offset.1st.estimated_number_max",_yawrate_offset_parameter.estimated_number_max);
      node->get_parameter("yawrate_offset.1st.estimated_number_max",_yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max "<<_yawrate_offset_parameter.estimated_number_max<<std::endl;
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "yawrate_offset_2nd";
      subscribe_topic_name = "heading_interpolate_2nd";
      node->declare_parameter("yawrate_offset.2nd.estimated_number_max",_yawrate_offset_parameter.estimated_number_max);
      node->get_parameter("yawrate_offset.2nd.estimated_number_max",_yawrate_offset_parameter.estimated_number_max);
      std::cout<< "estimated_number_max "<<_yawrate_offset_parameter.estimated_number_max<<std::endl;
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(),"Invalid argument");
      rclcpp::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "No arguments");
    rclcpp::shutdown();
  }

  auto sub1 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub2 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback);
  auto sub3 = node->create_subscription<eagleye_msgs::msg::Heading>(subscribe_topic_name, 1000, heading_interpolate_callback);
  auto sub4 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto sub5 = node->create_subscription<geometry_msgs::msg::TwistStamped>(comparison_twist_topic_name, rclcpp::QoS(10), comparison_velocity_callback);
  _pub = node->create_publisher<eagleye_msgs::msg::YawrateOffset>(publish_topic_name, rclcpp::QoS(10));

  rclcpp::spin(node);


  return 0;
}
