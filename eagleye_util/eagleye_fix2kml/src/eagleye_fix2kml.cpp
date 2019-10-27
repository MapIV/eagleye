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
 * eagleye_fix2kml
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "eagleye_msgs/Distance.h"
#include "eagleye_fix2kml/GoogleEarthPath.hpp"

static double interval = 0.2; //m
static double driving_distance = 0.0;
static double driving_distance_last = 0.0;

GoogleEarthPath path("eagleye_fix.kml","eagleye_fix");  //contains the simultaneously recorded bagfile name

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  driving_distance = msg->distance;
}

void receive_data(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  if( (driving_distance - driving_distance_last) > interval)
  {
    path.addPoint(msg->longitude,msg->latitude,msg->altitude);
    driving_distance_last = driving_distance;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eagleye_fix2kml");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/eagleye/fix", 1000, receive_data);
  ros::Subscriber sub2 = n.subscribe("/eagleye/distance", 1000, distance_callback);
  ros::spin();

  return 0;
}
