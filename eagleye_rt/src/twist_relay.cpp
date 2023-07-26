// Copyright (c) 2022, Map IV, Inc.
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
 * twist_relay.cpp
 * Author MapIV Sasaki
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class TwistRelay: public rclcpp::Node
{
public:
  TwistRelay();
  ~TwistRelay();

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_sub_;

  rclcpp::Clock clock_;
  rclcpp::Logger logger_;

  void twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void twist_with_covariance_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);

};

TwistRelay::TwistRelay() : Node("eagleye_twist_relay"),
    clock_(RCL_ROS_TIME),
    logger_(get_logger())
{
  int subscribe_twist_topic_type = 0;
  std::string subscribe_twist_topic_name = "/can_twist";
  std::string publish_twist_topic_name = "vehicle/twist";

  declare_parameter("twist.twist_type", subscribe_twist_topic_type);
  declare_parameter("twist.twist_topic", subscribe_twist_topic_name);

  get_parameter("twist.twist_type", subscribe_twist_topic_type);
  get_parameter("twist.twist_topic", subscribe_twist_topic_name);

  std::cout<< "subscribe_twist_topic_type: " << subscribe_twist_topic_type << std::endl;
  std::cout<< "subscribe_twist_topic_name: " << subscribe_twist_topic_name << std::endl;

  // TwistStamped : 0, TwistWithCovarianceStamped: 1
  if(subscribe_twist_topic_type == 0)
  {
    twist_sub_ =  create_subscription<geometry_msgs::msg::TwistStamped>(subscribe_twist_topic_name, rclcpp::QoS(10),
        std::bind(&TwistRelay::twist_callback, this, std::placeholders::_1));
  }
  else if(subscribe_twist_topic_type == 1)
  {
    twist_with_covariance_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(subscribe_twist_topic_name, rclcpp::QoS(10),
        std::bind(&TwistRelay::twist_with_covariance_callback, this, std::placeholders::_1));
  }
  else 
  {
    RCLCPP_ERROR(this->get_logger(),"Invalid twist topic type");
    rclcpp::shutdown();
  }

  pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(publish_twist_topic_name, rclcpp::QoS(10));
};

TwistRelay::~TwistRelay(){}; 

void TwistRelay::twist_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  pub_->publish(*msg);
};

void TwistRelay::twist_with_covariance_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = msg->header.stamp;
  twist.twist = msg->twist.twist;
  pub_->publish(twist);
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TwistRelay>());

  return 0;
}
