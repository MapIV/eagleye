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
 * enable_additional_rolling_node.cpp
 * Author MapIV Hoda
 */

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

static ros::Publisher pub1,pub2;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static eagleye_msgs::YawrateOffset yawrate_offset_2nd;
static eagleye_msgs::YawrateOffset yawrate_offset_stop;
static eagleye_msgs::Distance distance;
static geometry_msgs::PoseStamped localization_pose;
static eagleye_msgs::AngularVelocityOffset angular_velocity_offset_stop;
static sensor_msgs::Imu imu;

static eagleye_msgs::Rolling rolling_angle;
static eagleye_msgs::AccYOffset acc_y_offset;

struct EnableAdditionalRollingParameter rolling_parameter;
struct EnableAdditionalRollingStatus rolling_status;

void  velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr &msg)
{
  velocity_scale_factor = *msg;
}

void distance_callback(const eagleye_msgs::Distance::ConstPtr& msg)
{
  distance = *msg;
}

void  yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr &msg)
{
  yawrate_offset_2nd = *msg;
}

void  yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr &msg)
{
  yawrate_offset_stop = *msg;
}

void  localization_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  localization_pose = *msg;
}

void  angular_velocity_offset_stop_callback(const eagleye_msgs::AngularVelocityOffset::ConstPtr &msg)
{
  angular_velocity_offset_stop = *msg;
}

void  imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu = *msg;
  acc_y_offset.header = msg->header;
  acc_y_offset.header.frame_id = "imu";
  rolling_angle.header = msg->header;
  rolling_angle.header.frame_id = "base_link";
  enable_additional_rolling_estimate(velocity_scale_factor,yawrate_offset_2nd,yawrate_offset_stop,distance,imu,localization_pose,angular_velocity_offset_stop,rolling_parameter,&rolling_status,&rolling_angle,&acc_y_offset);
  pub1.publish(acc_y_offset);
  pub2.publish(rolling_angle);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "enable_additional_rolling");
  ros::NodeHandle n;

  std::string subscribe_imu_topic_name;
  std::string subscribe_localization_pose_topic_name;

  n.getParam("localization_pose_topic",subscribe_localization_pose_topic_name);
  n.getParam("imu_topic",subscribe_imu_topic_name);
  n.getParam("reverse_imu",rolling_parameter.reverse_imu);
  n.getParam("reverse_imu_angular_velocity_x",rolling_parameter.reverse_imu_angular_velocity_x);
  n.getParam("reverse_imu_linear_acceleration_y",rolling_parameter.reverse_imu_linear_acceleration_y);
  n.getParam("enable_additional_rolling/matching_update_distance",rolling_parameter.matching_update_distance);
  n.getParam("enable_additional_rolling/stop_judgment_velocity_threshold",rolling_parameter.stop_judgment_velocity_threshold);
  n.getParam("enable_additional_rolling/rolling_buffer_num",rolling_parameter.rolling_buffer_num);
  n.getParam("enable_additional_rolling/link_Time_stamp_parameter",rolling_parameter.link_Time_stamp_parameter);
  n.getParam("enable_additional_rolling/imu_buffer_num",rolling_parameter.imu_buffer_num);

  std::cout<< "subscribe_localization_pose_topic_name "<<subscribe_localization_pose_topic_name<<std::endl;
  std::cout<< "subscribe_imu_topic_name "<<subscribe_imu_topic_name<<std::endl;
  std::cout<< "reverse_imu "<<rolling_parameter.reverse_imu<<std::endl;
  std::cout<< "reverse_imu_angular_velocity_x "<<rolling_parameter.reverse_imu_angular_velocity_x<<std::endl;
  std::cout<< "reverse_imu_linear_acceleration_y "<<rolling_parameter.reverse_imu_linear_acceleration_y<<std::endl;
  std::cout<< "matching_update_distance" <<rolling_parameter.matching_update_distance<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold" <<rolling_parameter.stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "rolling_buffer_num" <<rolling_parameter.rolling_buffer_num<<std::endl;
  std::cout<< "link_Time_stamp_parameter" <<rolling_parameter.link_Time_stamp_parameter<<std::endl;
  std::cout<< "imu_buffer_num" <<rolling_parameter.imu_buffer_num<<std::endl;

  ros::Subscriber sub1 = n.subscribe("velocity_scale_factor", 1000, velocity_scale_factor_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = n.subscribe("yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("yawrate_offset_stop", 1000, yawrate_offset_stop_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = n.subscribe("distance", 1000, distance_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub5 = n.subscribe(subscribe_localization_pose_topic_name, 1000, localization_pose_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub6 = n.subscribe("angular_velocity_offset_stop", 1000, angular_velocity_offset_stop_callback , ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub7 = n.subscribe(subscribe_imu_topic_name, 1000, imu_callback , ros::TransportHints().tcpNoDelay());

  pub1 = n.advertise<eagleye_msgs::AccYOffset>("acc_y_offset_additional_rolling", 1000);
  pub2 = n.advertise<eagleye_msgs::Rolling>("enable_additional_rolling", 1000);
 
  ros::spin();

  return 0;
}
