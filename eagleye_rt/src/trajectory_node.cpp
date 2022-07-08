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
 * trajectory.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

static sensor_msgs::msg::Imu imu;
static geometry_msgs::msg::TwistStamped velocity;
static eagleye_msgs::msg::StatusStamped velocity_status;
static geometry_msgs::msg::TwistStamped correction_velocity;
static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Heading heading_interpolate_3rd;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::msg::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::msg::Pitching pitching;

static geometry_msgs::msg::Vector3Stamped enu_vel;
static eagleye_msgs::msg::Position enu_relative_pos;
static geometry_msgs::msg::TwistStamped eagleye_twist;
rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub1;
rclcpp::Publisher<eagleye_msgs::msg::Position>::SharedPtr pub2;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub3;
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub4;

struct TrajectoryParameter trajectory_parameter;
struct TrajectoryStatus trajectory_status;

static double timer_updata_rate = 10;
static double th_deadlock_time = 1;

static double imu_time_last,velocity_time_last;
static bool input_status;

static bool use_canless_mode;

void correction_velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  correction_velocity = *msg;
}

void velocity_status_callback(const eagleye_msgs::msg::StatusStamped::ConstPtr msg)
{
  velocity_status = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  velocity_scale_factor = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::msg::Heading::ConstSharedPtr msg)
{
  heading_interpolate_3rd = *msg;
}

void yawrate_offset_stop_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_stop = *msg;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::msg::YawrateOffset::ConstSharedPtr msg)
{
  yawrate_offset_2nd = *msg;
}

void pitching_callback(const eagleye_msgs::msg::Pitching::ConstSharedPtr msg)
{
  pitching = *msg;
}

void velocity_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  velocity = *msg;
}

void on_timer()
{
  rclcpp::Time imu_clock(imu.header.stamp);
  double imu_time = imu_clock.seconds();
  rclcpp::Time velocity_clock(velocity.header.stamp);
  double velocity_time = velocity_clock.seconds();
  if (std::abs(imu_time - imu_time_last) < th_deadlock_time &&
      std::abs(velocity_time - velocity_time_last) < th_deadlock_time &&
      std::abs(velocity_time - imu_time) < th_deadlock_time)
  {
    input_status = true;
  }
  else
  {
    input_status = false;
    RCLCPP_WARN(rclcpp::get_logger("trajectory"), "Twist is missing the required input topics.");
  }

  if (imu_time != imu_time_last) imu_time_last = imu_time;
  if (velocity_time != velocity_time_last) velocity_time_last = velocity_time;
}

void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if(use_canless_mode && !velocity_status.status.enabled_status) return;

  eagleye_msgs::msg::StatusStamped velocity_enable_status;
  if(use_canless_mode)
  {
    velocity_enable_status = velocity_status;
  }
  else
  {
    velocity_enable_status.header = velocity_scale_factor.header;
    velocity_enable_status.status = velocity_scale_factor.status;
  }

  imu = *msg;
  if(input_status)
  {
    enu_vel.header = msg->header;
    enu_vel.header.frame_id = "gnss";
    enu_relative_pos.header = msg->header;
    enu_relative_pos.header.frame_id = "base_link";
    eagleye_twist.header = msg->header;
    eagleye_twist.header.frame_id = "base_link";
    trajectory3d_estimate(imu,correction_velocity,velocity_enable_status,heading_interpolate_3rd,yawrate_offset_stop,yawrate_offset_2nd,pitching,
      trajectory_parameter,&trajectory_status,&enu_vel,&enu_relative_pos,&eagleye_twist);

    if (heading_interpolate_3rd.status.enabled_status)
    {
      pub1->publish(enu_vel);
      pub2->publish(enu_relative_pos);
    }
    pub3->publish(eagleye_twist);

    geometry_msgs::msg::TwistWithCovarianceStamped eagleye_twist_with_covariance;
    eagleye_twist_with_covariance.header = msg->header;
    eagleye_twist_with_covariance.header.frame_id = "base_link";
    eagleye_twist_with_covariance.twist.twist = eagleye_twist.twist;
    // TODO(Map IV): temporary value
    // linear.y, linear.z, angular.x, and angular.y are not calculated values.
    eagleye_twist_with_covariance.twist.covariance[0] = 0.2 * 0.2;
    eagleye_twist_with_covariance.twist.covariance[7] = 10000.0;
    eagleye_twist_with_covariance.twist.covariance[14] = 10000.0;
    eagleye_twist_with_covariance.twist.covariance[21] = 10000.0;
    eagleye_twist_with_covariance.twist.covariance[28] = 10000.0;
    eagleye_twist_with_covariance.twist.covariance[35] = 0.1 * 0.1;

    pub4->publish(eagleye_twist_with_covariance);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trajectory");

  node->declare_parameter("trajectory.stop_judgment_velocity_threshold",trajectory_parameter.stop_judgment_velocity_threshold);
  node->declare_parameter("trajectory.stop_judgment_yawrate_threshold",trajectory_parameter.stop_judgment_yawrate_threshold);
  node->declare_parameter("timer_updata_rate",timer_updata_rate);
  node->declare_parameter("th_deadlock_time",th_deadlock_time);
  node->declare_parameter("use_canless_mode",use_canless_mode);

  node->get_parameter("trajectory.stop_judgment_velocity_threshold",trajectory_parameter.stop_judgment_velocity_threshold);
  node->get_parameter("trajectory.stop_judgment_yawrate_threshold",trajectory_parameter.stop_judgment_yawrate_threshold);
  node->get_parameter("timer_updata_rate",timer_updata_rate);
  node->get_parameter("th_deadlock_time",th_deadlock_time);
  node->get_parameter("use_canless_mode",use_canless_mode);

  std::cout<< "stop_judgment_velocity_threshold "<<trajectory_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "stop_judgment_yawrate_threshold "<<trajectory_parameter.stop_judgment_yawrate_threshold<<std::endl;
  std::cout<< "timer_updata_rate "<<timer_updata_rate<<std::endl;
  std::cout<< "th_deadlock_time "<<th_deadlock_time<<std::endl;
  std::cout<< "use_canless_mode "<<use_canless_mode<<std::endl;

  auto sub1 = node->create_subscription<sensor_msgs::msg::Imu>("imu/data_tf_converted", 1000, imu_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub2 = node->create_subscription<geometry_msgs::msg::TwistStamped>("velocity", rclcpp::QoS(10), velocity_callback);
  auto sub3 = node->create_subscription<eagleye_msgs::msg::StatusStamped>("velocity_status", rclcpp::QoS(10), velocity_status_callback);
  auto sub4 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub5 = node->create_subscription<eagleye_msgs::msg::Heading>("heading_interpolate_3rd", rclcpp::QoS(10), heading_interpolate_3rd_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub6 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_stop", rclcpp::QoS(10), yawrate_offset_stop_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub7 = node->create_subscription<eagleye_msgs::msg::YawrateOffset>("yawrate_offset_2nd", rclcpp::QoS(10), yawrate_offset_2nd_callback);  //ros::TransportHints().tcpNoDelay()
  auto sub8 = node->create_subscription<eagleye_msgs::msg::Pitching>("pitching", rclcpp::QoS(10), pitching_callback);  //ros::TransportHints().tcpNoDelay()
  pub1 = node->create_publisher<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000);
  pub2 = node->create_publisher<eagleye_msgs::msg::Position>("enu_relative_pos", 1000);
  pub3 = node->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1000);
  pub4 = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_with_covariance", 1000);

  double delta_time = 1.0 / static_cast<double>(timer_updata_rate);
  auto timer_callback = std::bind(on_timer);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  auto timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    node->get_clock(), period_ns, std::move(timer_callback),
    node->get_node_base_interface()->get_context());
  node->get_node_timers_interface()->add_timer(timer, nullptr);

  rclcpp::spin(node);

  return 0;
}
