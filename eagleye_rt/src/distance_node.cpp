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
 * distance.cpp
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static eagleye_msgs::msg::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::msg::Distance distance;

struct DistanceStatus distance_status;

rclcpp::Publisher<eagleye_msgs::msg::Distance>::SharedPtr pub;

void velocity_scale_factor_callback(const eagleye_msgs::msg::VelocityScaleFactor::ConstSharedPtr msg)
{
  distance.header = msg->header;
  distance.header.frame_id = "base_link";
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
  distance_estimate(velocity_scale_factor,&distance_status,&distance);
  if(distance_status.time_last != 0)
  {
    pub->publish(distance);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("distance");

  auto sub1 = node->create_subscription<eagleye_msgs::msg::VelocityScaleFactor>("eagleye/velocity_scale_factor", rclcpp::QoS(10), velocity_scale_factor_callback);
  pub = node->create_publisher<eagleye_msgs::msg::Distance>("eagleye/distance", rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
