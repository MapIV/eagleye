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
 #include "coordinate.hpp"
 #include "navigation.hpp"

 // #include "eagleye_msgs/AccXScaleFactor.h"
 // #include "eagleye_msgs/AccXOffset.h"
 // #include "eagleye_msgs/Height.h"
 // #include "eagleye_msgs/Pitching.h"

 static sensor_msgs::Imu imu;
 static sensor_msgs::NavSatFix fix;
 static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
 static eagleye_msgs::Distance distance;

 static ros::Publisher pub1,pub2,pub3,pub4;
 static eagleye_msgs::Height height;
 static eagleye_msgs::Pitching pitching;
 static eagleye_msgs::AccXOffset acc_x_offset;
 static eagleye_msgs::AccXScaleFactor acc_x_scale_factor;

 struct HeightParameter height_parameter;
 struct HeightStatus height_status;



// void nmea_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
// {
//   nmea.header = msg->header;
//   nmea.utc_seconds = msg->utc_seconds;
//   nmea.alt = msg->alt;
//   nmea.gps_qual = msg->gps_qual;
//   nmea.undulation = msg->undulation;
// }

void fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  fix.header = msg->header;
  fix.status = msg->status;
  fix.latitude = msg->latitude;
  fix.longitude = msg->longitude;
  fix.altitude = msg->altitude;
  fix.position_covariance = msg->position_covariance;
  fix.position_covariance_type = msg->position_covariance_type;
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance.header = msg->header;
  distance.distance = msg->distance;
  distance.status = msg->status;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu.header = msg->header;
  imu.orientation = msg->orientation;
  imu.orientation_covariance = msg->orientation_covariance;
  imu.angular_velocity = msg->angular_velocity;
  imu.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu.linear_acceleration = msg->linear_acceleration;
  imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
  height.header = msg->header;
  pitching.header = msg->header;
  acc_x_offset.header = msg->header;
  acc_x_scale_factor.header = msg->header;
  pitching_estimate(imu,fix,velocity_scale_factor,distance,height_parameter,&height_status,&height,&pitching,&acc_x_offset,&acc_x_scale_factor);
  pub1.publish(height);
  pub2.publish(pitching);
  pub3.publish(acc_x_offset);
  pub4.publish(acc_x_scale_factor);
  height.status.estimate_status = false;
  pitching.status.estimate_status = false;
  acc_x_offset.status.estimate_status = false;
  acc_x_scale_factor.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height");
  ros::NodeHandle n;

  n.getParam("/eagleye/height/estimated_distance",height_parameter.estimated_distance);
  n.getParam("/eagleye/height/estimated_distance_max",height_parameter.estimated_distance_max);
  n.getParam("/eagleye/height/separation_distance",height_parameter.separation_distance);
  n.getParam("/eagleye/height/estimated_velocity_threshold",height_parameter.estimated_velocity_threshold);
  n.getParam("/eagleye/height/estimated_velocity_coefficient",height_parameter.estimated_velocity_coefficient);
  n.getParam("/eagleye/height/estimated_height_coefficient",height_parameter.estimated_height_coefficient);
  n.getParam("/eagleye/height/outlier_threshold",height_parameter.outlier_threshold);
  n.getParam("/eagleye/height/average_num",height_parameter.average_num);

  std::cout<< "estimated_distance "<<height_parameter.estimated_distance<<std::endl;
  std::cout<< "estimated_distance_max "<<height_parameter.estimated_distance_max<<std::endl;
  std::cout<< "separation_distance "<<height_parameter.separation_distance<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<height_parameter.estimated_velocity_threshold<<std::endl;
  std::cout<< "estimated_velocity_coefficient "<<height_parameter.estimated_velocity_coefficient<<std::endl;
  std::cout<< "estimated_height_coefficient "<<height_parameter.estimated_height_coefficient<<std::endl;
  std::cout<< "outlier_threshold "<<height_parameter.outlier_threshold<<std::endl;
  std::cout<< "average_num "<<height_parameter.average_num<<std::endl;

  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe(subscribe_topic_name, 1000, fix_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("/eagleye/distance", 1000, distance_callback, ros::TransportHints().tcpNoDelay());


  pub1 = n.advertise<eagleye_msgs::Height>("/eagleye/height", 1000);
  pub2 = n.advertise<eagleye_msgs::Pitching>("/eagleye/pitching", 1000);
  pub3 = n.advertise<eagleye_msgs::AccXOffset>("/eagleye/acc_x_offset", 1000);
  pub4 = n.advertise<eagleye_msgs::AccXScaleFactor>("/eagleye/acc_x_scale_factor", 1000);

  ros::spin();

  return 0;
}
