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
 * velocity_scale_factor.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static rtklib_msgs::msg::RtklibNav _rtklib_nav;
static nmea_msgs::msg::Gprmc _nmea_rmc;
static geometry_msgs::msg::TwistStamped _velocity;
static sensor_msgs::msg::Imu _imu;
static geometry_msgs::msg::TwistStamped _correction_velocity;

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _pub1;
rclcpp::Publisher<eagleye_msgs::msg::VelocityScaleFactor>::SharedPtr _pub2;
static eagleye_msgs::msg::VelocityScaleFactor _velocity_scale_factor;

struct VelocityScaleFactorParameter _velocity_scale_factor_parameter;
struct VelocityScaleFactorStatus _velocity_scale_factor_status;

std::string _use_gnss_mode;

bool _is_first_move = false;

std::string _velocity_scale_factor_save_str;
double _saved_vsf_estimater_number;
double _saved_velocity_scale_factor = 1.0;
double _previous_velocity_scale_factor = 1.0;
double _th_velocity_scale_factor_percent = 20;

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg)
{
  _rtklib_nav = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  _velocity = *msg;

  if (_is_first_move == false && msg->twist.linear.x > _velocity_scale_factor_parameter.moving_judgment_threshold)
  {
    _is_first_move = true;
  }
}

void rmc_callback(const nmea_msgs::msg::Gprmc::ConstSharedPtr msg)
{
  _nmea_rmc = *msg;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  double initial_velocity_scale_factor = _saved_velocity_scale_factor;

  _imu = *msg;
  _velocity_scale_factor.header = msg->header;
  _velocity_scale_factor.header.frame_id = "base_link";

  _correction_velocity.header = msg->header;
  _correction_velocity.header.frame_id = "base_link";

  if (_is_first_move == false)
  {
    _velocity_scale_factor.scale_factor = initial_velocity_scale_factor;
    _correction_velocity.twist = _velocity.twist;
    _pub1->publish(_correction_velocity);
    _pub2->publish(_velocity_scale_factor);
    return;
  }

  if (_use_gnss_mode == "rtklib" || _use_gnss_mode == "RTKLIB") // use RTKLIB mode
  {
    velocity_scale_factor_estimate(_rtklib_nav,_velocity,_velocity_scale_factor_parameter,
      &_velocity_scale_factor_status,&_correction_velocity,&_velocity_scale_factor);
  }
  else if (_use_gnss_mode == "nmea" || _use_gnss_mode == "NMEA") // use NMEA mode
  {
    velocity_scale_factor_estimate(_nmea_rmc,_velocity,_velocity_scale_factor_parameter,
      &_velocity_scale_factor_status,&_correction_velocity,&_velocity_scale_factor);
  }

  _velocity_scale_factor.status.is_abnormal = false;
  if (!std::isfinite(_velocity_scale_factor.scale_factor)) {
    _correction_velocity.twist.linear.x = _velocity.twist.linear.x * _previous_velocity_scale_factor;
    _velocity_scale_factor.scale_factor = _previous_velocity_scale_factor;
    _velocity_scale_factor.status.is_abnormal = true;
    _velocity_scale_factor.status.error_code = eagleye_msgs::msg::Status::NAN_OR_INFINITE;
  }
  else if (_th_velocity_scale_factor_percent / 100 < std::abs(1.0 - _velocity_scale_factor.scale_factor))
  {
    _correction_velocity.twist.linear.x = _velocity.twist.linear.x * _previous_velocity_scale_factor;
    _velocity_scale_factor.scale_factor = _previous_velocity_scale_factor;
    _velocity_scale_factor.status.is_abnormal = true;
    _velocity_scale_factor.status.error_code = eagleye_msgs::msg::Status::TOO_LARGE_OR_SMALL;
  }
  else
  {
    _previous_velocity_scale_factor = _velocity_scale_factor.scale_factor;
  }

  _pub1->publish(_correction_velocity);
  _pub2->publish(_velocity_scale_factor);
}

void load_velocity_scale_factor(std::string txt_path)
{
  std::ifstream ifs(txt_path);
  if (!ifs)
  {
    std::cout << "Initial VelocityScaleFactor file not found!" << std::endl;
  }
  else
  {
    std::cout << "Loaded the saved velocity scale factor!" << std::endl;
    int count = 0;
    std::string row;
    while (getline(ifs, row))
    {
      if(count == 1)
      {
        _saved_vsf_estimater_number = std::stod(row);
        std::cout<< "saved_vsf_estimater_number " << _saved_vsf_estimater_number << std::endl;
      }
      if(count == 3)
      {
        _saved_velocity_scale_factor = std::stod(row);
        _velocity_scale_factor_status.estimate_start_status = true;
        _velocity_scale_factor_status.velocity_scale_factor_last = _saved_velocity_scale_factor;
        _velocity_scale_factor.status.enabled_status = true;
        _velocity_scale_factor.scale_factor = _saved_velocity_scale_factor;
        std::cout<< "saved_velocity_scale_factor " << _saved_velocity_scale_factor << std::endl;
      }
      count++;
    }
  }
  ifs.close();
}

void on_timer()
{
  if(!_velocity_scale_factor.status.enabled_status && _saved_vsf_estimater_number >= _velocity_scale_factor_status.estimated_number)
  {
    std::ofstream csv_file(_velocity_scale_factor_save_str);
    return;
  }

  std::ofstream csv_file(_velocity_scale_factor_save_str);
  csv_file << "estimated_number";
  csv_file << "\n";
  csv_file << _velocity_scale_factor_status.estimated_number;
  csv_file << "\n";
  csv_file << "velocity_scale_factor";
  csv_file << "\n";
  csv_file << _velocity_scale_factor_status.velocity_scale_factor_last;
  csv_file << "\n";
  csv_file.close();

  _saved_vsf_estimater_number = _velocity_scale_factor_status.estimated_number;

  return;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("eagleye_velocity_scale_factor");

  double velocity_scale_factor_save_duration = 100.0;

  std::string subscribe_twist_topic_name = "vehicle/twist";

  std::string subscribe_rtklib_nav_topic_name = "gnss/rtklib_nav";
  std::string subscribe_rmc_topic_name = "gnss/rmc";

  std::string yaml_file;
  node->declare_parameter("yaml_file",yaml_file);
  node->get_parameter("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _use_gnss_mode = conf["/**"]["ros__parameters"]["use_gnss_mode"].as<std::string>();

    _velocity_scale_factor_parameter.imu_rate = conf["/**"]["ros__parameters"]["common"]["imu_rate"].as<double>();
    _velocity_scale_factor_parameter.gnss_rate = conf["/**"]["ros__parameters"]["common"]["gnss_rate"].as<double>();
    _velocity_scale_factor_parameter.moving_judgment_threshold = conf["/**"]["ros__parameters"]["common"]["stop_judgment_threshold"].as<double>();

    _velocity_scale_factor_parameter.estimated_minimum_interval = conf["/**"]["ros__parameters"]["velocity_scale_factor"]["estimated_minimum_interval"].as<double>();
    _velocity_scale_factor_parameter.estimated_maximum_interval = conf["/**"]["ros__parameters"]["velocity_scale_factor"]["estimated_maximum_interval"].as<double>();
    _velocity_scale_factor_parameter.gnss_receiving_threshold = conf["/**"]["ros__parameters"]["velocity_scale_factor"]["gnss_receiving_threshold"].as<double>();

    node->declare_parameter("velocity_scale_factor_save_str",_velocity_scale_factor_save_str);
    node->declare_parameter("velocity_scale_factor.save_velocity_scale_factor",_velocity_scale_factor_parameter.save_velocity_scale_factor);
    node->declare_parameter("velocity_scale_factor.velocity_scale_factor_save_duration",velocity_scale_factor_save_duration);
    node->declare_parameter("velocity_scale_factor.th_velocity_scale_factor_percent",_th_velocity_scale_factor_percent);

    node->get_parameter("velocity_scale_factor_save_str",_velocity_scale_factor_save_str);
    node->get_parameter("velocity_scale_factor.save_velocity_scale_factor",_velocity_scale_factor_parameter.save_velocity_scale_factor);
    node->get_parameter("velocity_scale_factor.velocity_scale_factor_save_duration",velocity_scale_factor_save_duration);
    node->get_parameter("velocity_scale_factor.th_velocity_scale_factor_percent",_th_velocity_scale_factor_percent);

    std::cout << "use_gnss_mode " << _use_gnss_mode << std::endl;

    std::cout << "subscribe_twist_topic_name " << subscribe_twist_topic_name << std::endl;
    std::cout << "subscribe_rtklib_nav_topic_name " << subscribe_rtklib_nav_topic_name << std::endl;

    std::cout << "gnss_rate " << _velocity_scale_factor_parameter.gnss_rate << std::endl;
    std::cout << "moving_judgment_threshold " << _velocity_scale_factor_parameter.moving_judgment_threshold << std::endl;

    std::cout << "estimated_minimum_interval " << _velocity_scale_factor_parameter.estimated_minimum_interval << std::endl;
    std::cout << "estimated_maximum_interval " << _velocity_scale_factor_parameter.estimated_maximum_interval << std::endl;
    std::cout << "gnss_receiving_threshold " << _velocity_scale_factor_parameter.gnss_receiving_threshold << std::endl;

    std::cout<< "velocity_scale_factor_save_str " << _velocity_scale_factor_save_str << std::endl;
    std::cout<< "save_velocity_scale_factor " << _velocity_scale_factor_parameter.save_velocity_scale_factor << std::endl;
    std::cout<< "velocity_scale_factor_save_duration " << velocity_scale_factor_save_duration << std::endl;
    std::cout<< "th_velocity_scale_factor_percent "<<_th_velocity_scale_factor_percent<<std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mvelocity_scale_factor Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }
  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);
  auto sub2 = node->create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, 1000, velocity_callback);
  auto sub3 = node->create_subscription<rtklib_msgs::msg::RtklibNav>(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback);
  auto sub4 = node->create_subscription<nmea_msgs::msg::Gprmc>(subscribe_rmc_topic_name, 1000, rmc_callback);
  _pub1 = node->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10));
  _pub2 = node->create_publisher<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10));

  double delta_time = static_cast<double>(velocity_scale_factor_save_duration);
  auto timer_callback = std::bind(on_timer);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  auto timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    node->get_clock(), period_ns, std::move(timer_callback),
    node->get_node_base_interface()->get_context());
  if(_velocity_scale_factor_parameter.save_velocity_scale_factor)
  {
    node->get_node_timers_interface()->add_timer(timer, nullptr);
    load_velocity_scale_factor(_velocity_scale_factor_save_str);
  }

  rclcpp::spin(node);

  return 0;
}
