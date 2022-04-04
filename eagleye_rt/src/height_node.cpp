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
 * height_node.cpp
 * Author MapIV  Takanose
 */

 #include "ros/ros.h"
 #include "coordinate/coordinate.hpp"
 #include "navigation/navigation.hpp"

 static sensor_msgs::Imu imu;
 static nmea_msgs::Gpgga gga;
 static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
 static eagleye_msgs::Distance distance;

 static ros::Publisher pub1,pub2,pub3,pub4,pub5;
 static eagleye_msgs::Height height;
 static eagleye_msgs::Pitching pitching;
 static eagleye_msgs::AccXOffset acc_x_offset;
 static eagleye_msgs::AccXScaleFactor acc_x_scale_factor;

 struct HeightParameter height_parameter;
 struct HeightStatus height_status;

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  gga = *msg;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu = *msg;
  height.header = msg->header;
  height.header.frame_id = "base_link";
  pitching.header = msg->header;
  pitching.header.frame_id = "base_link";
  acc_x_offset.header = msg->header;
  acc_x_scale_factor.header = msg->header;
  pitching_estimate(imu,gga,velocity_scale_factor,distance,height_parameter,&height_status,&height,&pitching,&acc_x_offset,&acc_x_scale_factor);
  pub1.publish(height);
  pub2.publish(pitching);
  pub3.publish(acc_x_offset);
  pub4.publish(acc_x_scale_factor);

  if(height_status.flag_reliability)
  {
    pub5.publish(gga);
  }

  height_status.flag_reliability = false;
  height.status.estimate_status = false;
  pitching.status.estimate_status = false;
  acc_x_offset.status.estimate_status = false;
  acc_x_scale_factor.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height");
  ros::NodeHandle n;

  std::string subscribe_navsatgga_topic_name = "/navsat/gga";
  std::string subscribe_imu_topic_name = "/imu/data_raw";

  n.getParam("navsatgga_topic",subscribe_navsatgga_topic_name);
  n.getParam("imu_topic",subscribe_imu_topic_name);
  n.getParam("height/estimated_distance",height_parameter.estimated_distance);
  n.getParam("height/estimated_distance_max",height_parameter.estimated_distance_max);
  n.getParam("height/separation_distance",height_parameter.separation_distance);
  n.getParam("height/estimated_velocity_threshold",height_parameter.estimated_velocity_threshold);
  n.getParam("height/estimated_velocity_coefficient",height_parameter.estimated_velocity_coefficient);
  n.getParam("height/estimated_height_coefficient",height_parameter.estimated_height_coefficient);
  n.getParam("height/outlier_threshold",height_parameter.outlier_threshold);
  n.getParam("height/average_num",height_parameter.average_num);

  std::cout<< "subscribe_navsatgga_topic_name "<<subscribe_navsatgga_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "estimated_distance "<<height_parameter.estimated_distance<<std::endl;
  std::cout<< "estimated_distance_max "<<height_parameter.estimated_distance_max<<std::endl;
  std::cout<< "separation_distance "<<height_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<height_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_velocity_coefficient "<<height_parameter.estimated_velocity_coefficient<<std::endl;
  std::cout<< "estimated_height_coefficient "<<height_parameter.estimated_height_coefficient<<std::endl;
  std::cout<< "outlier_threshold "<<height_parameter.outlier_threshold<<std::endl;
  std::cout<< "average_num "<<height_parameter.average_num<<std::endl;

  ros::Subscriber sub1 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_navsatgga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());

  pub1 = n.advertise<eagleye_msgs::Height>("height", 1000);
  pub2 = n.advertise<eagleye_msgs::Pitching>("pitching", 1000);
  pub3 = n.advertise<eagleye_msgs::AccXOffset>("acc_x_offset", 1000);
  pub4 = n.advertise<eagleye_msgs::AccXScaleFactor>("acc_x_scale_factor", 1000);
  pub5 = n.advertise<sensor_msgs::NavSatFix>("navsat/reliability_gga", 1000);

  ros::spin();

  return 0;
}
