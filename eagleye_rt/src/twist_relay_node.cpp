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
 * correction_imu.cpp
 * Author MapIV Sasaki
 */

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"


class TwistRelay
{
public:
  TwistRelay();
  ~TwistRelay();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_, sub2_;

  void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void twist_with_covariance_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

};

TwistRelay::TwistRelay()
{
  int subscribe_twist_topic_type = 0;
  std::string subscribe_twist_topic_name = "/can_twist";
  std::string publish_twist_topic_name = "vehicle/twist";

  std::string yaml_file;
  nh_.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    subscribe_twist_topic_type = conf["twist"]["twist_type"].as<int>();
    subscribe_twist_topic_name = conf["twist"]["twist_topic"].as<std::string>();
    std::cout<< "subscribe_twist_topic_type: " << subscribe_twist_topic_type << std::endl;
    std::cout<< "subscribe_twist_topic_name: " << subscribe_twist_topic_name << std::endl;

  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;twist_relay Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  // TwistStamped : 0, TwistWithCovarianceStamped: 1
  if(subscribe_twist_topic_type == 0)
  {
        sub_ = nh_.subscribe(subscribe_twist_topic_name, 1000, &TwistRelay::twist_callback, this, ros::TransportHints().tcpNoDelay());
  }
  else if(subscribe_twist_topic_type == 1)
  {
        sub2_ = nh_.subscribe(subscribe_twist_topic_name, 1000, &TwistRelay::twist_with_covariance_callback, this, ros::TransportHints().tcpNoDelay());
  }
  else 
  {
    ROS_ERROR("Invalid twist topic type");
    ros::shutdown();
  }

  pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vehicle/twist", 1000);
};

TwistRelay::~TwistRelay(){}; 

void TwistRelay::twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  pub_.publish(*msg);
};

void TwistRelay::twist_with_covariance_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  geometry_msgs::TwistStamped twist;
  twist.header.stamp = msg->header.stamp;
  twist.twist = msg->twist.twist;
  pub_.publish(twist);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_relay");
  
  TwistRelay main_node;

  ros::spin();

  return 0;
}
