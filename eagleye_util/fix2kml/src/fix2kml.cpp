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
 * fix2kml.cpp
 * Author MapIV Sekino
 */

#include "eagleye_msgs/msg/distance.hpp"
#include "fix2kml/KmlGenerator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

static double interval = 0.2;  //m
static double driving_distance = 0.0;
static double driving_distance_last = 0.0;
static std::string filename, kmlname, fixname, color = "ff0000ff";

void distance_callback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
{
  driving_distance = msg->distance;
}

void receive_data(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg, KmlGenerator* kmlfile)
{
  if ((driving_distance - driving_distance_last) > interval) {
    kmlfile->addPoint(msg->longitude, msg->latitude, msg->altitude);
    kmlfile->KmlGenerate(filename);
    driving_distance_last = driving_distance;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fix2kml");

  node->declare_parameter("filename", filename);
  node->declare_parameter("kmlname", kmlname);
  node->declare_parameter("fixname", fixname);
  node->declare_parameter("color", color);

  node->get_parameter("filename", filename);
  node->get_parameter("kmlname", kmlname);
  node->get_parameter("fixname", fixname);
  node->get_parameter("color", color);

  std::cout << "filename: " << filename << std::endl;
  std::cout << "kmlname: " << kmlname << std::endl;
  std::cout << "fixname: " << fixname << std::endl;
  std::cout << "color: " << color << std::endl;

  KmlGenerator kmlfile(kmlname,color);

  std::function<void(std::shared_ptr<sensor_msgs::msg::NavSatFix>)> sub1_fnc = std::bind(&receive_data, std::placeholders::_1, &kmlfile);
  auto sub1 = node->create_subscription<sensor_msgs::msg::NavSatFix>(fixname, rclcpp::QoS(10), sub1_fnc);
  auto sub2 = node->create_subscription<eagleye_msgs::msg::Distance>("/eagleye/distance", rclcpp::QoS(10), distance_callback);
  rclcpp::spin(node);

  return 0;
}
