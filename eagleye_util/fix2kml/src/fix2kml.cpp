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

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "eagleye_msgs/Distance.h"
#include "fix2kml/KmlGenerator.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <functional>

class Fix2Kml
{
public:
  Fix2Kml(ros::NodeHandle &nh);
  ~Fix2Kml();

private:
  void distanceCallback(const eagleye_msgs::Distance::ConstPtr &msg);
  void receiveData(const sensor_msgs::NavSatFix::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub1_, sub2_;
  std::shared_ptr<KmlGenerator> kmlfile_;
  double interval_;
  double driving_distance_;
  double driving_distance_last_;
  std::string filename_, kmlname_, fixname_, color_;
};

Fix2Kml::Fix2Kml(ros::NodeHandle &nh) : nh_(nh)
{
  nh_.getParam("fix2kml/filename", filename_);
  nh_.getParam("fix2kml/kmlname", kmlname_);
  nh_.getParam("fix2kml/fixname", fixname_);
  nh_.getParam("fix2kml/color", color_);

  std::cout << "filename " << filename_ << std::endl;
  std::cout << "kmlname " << kmlname_ << std::endl;
  std::cout << "fixname " << fixname_ << std::endl;
  std::cout << "color " << color_ << std::endl;

  interval_ = 0.2; // m
  driving_distance_ = 0.0;
  driving_distance_last_ = 0.0;

  kmlfile_ = std::make_shared<KmlGenerator>(kmlname_, color_);

  sub1_ = nh_.subscribe<sensor_msgs::NavSatFix>(fixname_, 1000, std::bind(&Fix2Kml::receiveData, this, std::placeholders::_1));
  sub2_ = nh_.subscribe<eagleye_msgs::Distance>("eagleye/distance", 1000, std::bind(&Fix2Kml::distanceCallback, this, std::placeholders::_1));
}

Fix2Kml::~Fix2Kml()
{
}

void Fix2Kml::distanceCallback(const eagleye_msgs::Distance::ConstPtr &msg)
{
  driving_distance_ = msg->distance;
}

void Fix2Kml::receiveData(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  if ((driving_distance_ - driving_distance_last_) > interval_)
  {
    kmlfile_->addPoint(msg->longitude, msg->latitude, msg->altitude);
    kmlfile_->KmlGenerate(filename_);
    driving_distance_last_ = driving_distance_;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2kml");
  ros::NodeHandle nh;

  Fix2Kml fix2kml(nh);
  ros::spin();

  return 0;
}
