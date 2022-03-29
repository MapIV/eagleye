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
 * rtk_deadreckoning.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void rtk_deadreckoning_estimate(rtklib_msgs::msg::RtklibNav rtklib_nav,geometry_msgs::msg::Vector3Stamped enu_vel, sensor_msgs::msg::NavSatFix fix,  eagleye_msgs::msg::Heading heading, RtkDeadreckoningParameter rtk_deadreckoning_parameter, RtkDeadreckoningStatus* rtk_deadreckoning_status, eagleye_msgs::msg::Position* enu_absolute_rtk_deadreckoning,sensor_msgs::msg::NavSatFix* eagleye_fix)
{

  double enu_pos[3],enu_rtk[3];
  double ecef_base_pos[3];
  double ecef_rtk[3];
  double llh_pos[3],llh_rtk[3];

  rclcpp::Time ros_clock(rtklib_nav.header.stamp);
  rclcpp::Time ros_clock2(fix.header.stamp);
  rclcpp::Time ros_clock3(enu_vel.header.stamp);

  auto rtklib_nav_time = ros_clock.seconds();
  auto fix_time = ros_clock2.seconds();
  auto enu_vel_time = ros_clock3.seconds();

  if(rtk_deadreckoning_parameter.use_ecef_base_position)
  {
    enu_absolute_rtk_deadreckoning->ecef_base_pos.x = rtk_deadreckoning_parameter.ecef_base_pos_x;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.y = rtk_deadreckoning_parameter.ecef_base_pos_y;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.z = rtk_deadreckoning_parameter.ecef_base_pos_z;
    rtk_deadreckoning_status->ecef_base_pos_status = true;
    rtk_deadreckoning_status->position_estimate_start_status = true;
  }
  else if(!rtk_deadreckoning_status->ecef_base_pos_status && rtklib_nav_time!= 0)
  {
    enu_absolute_rtk_deadreckoning->ecef_base_pos.x = rtklib_nav.ecef_pos.x;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.y = rtklib_nav.ecef_pos.y;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.z = rtklib_nav.ecef_pos.z;
    rtk_deadreckoning_status->ecef_base_pos_status = true;
    rtk_deadreckoning_status->position_estimate_start_status = true;
  }

  if(rtk_deadreckoning_status->position_estimate_start_status)
  {
    ecef_base_pos[0] = enu_absolute_rtk_deadreckoning->ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_rtk_deadreckoning->ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_rtk_deadreckoning->ecef_base_pos.z;

    llh_rtk[0] = fix.latitude *M_PI/180;
    llh_rtk[1] = fix.longitude *M_PI/180;
    llh_rtk[2] = fix.altitude;

    llh2xyz(llh_rtk,ecef_rtk);
    xyz2enu(ecef_rtk,ecef_base_pos,enu_rtk);

    geometry_msgs::msg::PoseStamped pose;

    pose.pose.position.x = enu_rtk[0];
    pose.pose.position.y = enu_rtk[1];
    pose.pose.position.z = enu_rtk[2];

    heading.heading_angle = fmod(heading.heading_angle,2*M_PI);
    tf2::Transform transform;
    tf2::Quaternion q;
    transform.setOrigin(tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, (90* M_PI / 180)-heading.heading_angle);
    transform.setRotation(q);

    tf2::Transform transform2;
    tf2::Quaternion q2(rtk_deadreckoning_parameter.tf_gnss_rotation_x,rtk_deadreckoning_parameter.tf_gnss_rotation_y,rtk_deadreckoning_parameter.tf_gnss_rotation_z,rtk_deadreckoning_parameter.tf_gnss_rotation_w);
    transform2.setOrigin(transform*tf2::Vector3(-rtk_deadreckoning_parameter.tf_gnss_translation_x, -rtk_deadreckoning_parameter.tf_gnss_translation_y,-rtk_deadreckoning_parameter.tf_gnss_translation_z));
    transform2.setRotation(transform*q2);

    tf2::Vector3 tmp_pos;
    tmp_pos = transform2.getOrigin();

    enu_rtk[0] = tmp_pos.getX();
    enu_rtk[1] = tmp_pos.getY();
    enu_rtk[2] = tmp_pos.getZ();

    if (rtk_deadreckoning_status->position_stamp_last != fix_time && fix.status.status == 0)
    {
      rtk_deadreckoning_status->provisional_enu_pos_x = enu_rtk[0];
      rtk_deadreckoning_status->provisional_enu_pos_y = enu_rtk[1];
      rtk_deadreckoning_status->provisional_enu_pos_z = enu_rtk[2];
      enu_absolute_rtk_deadreckoning->status.enabled_status = true;
      enu_absolute_rtk_deadreckoning->status.estimate_status = true;
    }
    else if(rtk_deadreckoning_status->time_last != 0 && sqrt((enu_vel.vector.x * enu_vel.vector.x) + (enu_vel.vector.y * enu_vel.vector.y) + (enu_vel.vector.z * enu_vel.vector.z)) > rtk_deadreckoning_parameter.stop_judgment_velocity_threshold)
    {
      rtk_deadreckoning_status->provisional_enu_pos_x = enu_absolute_rtk_deadreckoning->enu_pos.x + enu_vel.vector.x * (enu_vel_time - rtk_deadreckoning_status->time_last);
      rtk_deadreckoning_status->provisional_enu_pos_y = enu_absolute_rtk_deadreckoning->enu_pos.y + enu_vel.vector.y * (enu_vel_time - rtk_deadreckoning_status->time_last);
      rtk_deadreckoning_status->provisional_enu_pos_z = enu_absolute_rtk_deadreckoning->enu_pos.z + enu_vel.vector.z * (enu_vel_time - rtk_deadreckoning_status->time_last);
      enu_absolute_rtk_deadreckoning->status.enabled_status = true;
      enu_absolute_rtk_deadreckoning->status.estimate_status = false;
    }

    enu_pos[0] = rtk_deadreckoning_status->provisional_enu_pos_x;
    enu_pos[1] = rtk_deadreckoning_status->provisional_enu_pos_y;
    enu_pos[2] = rtk_deadreckoning_status->provisional_enu_pos_z;

    enu2llh(enu_pos, ecef_base_pos, llh_pos);

    eagleye_fix->latitude = llh_pos[0] * 180/M_PI;
    eagleye_fix->longitude = llh_pos[1] * 180/M_PI;
    eagleye_fix->altitude = llh_pos[2];

    enu_absolute_rtk_deadreckoning->enu_pos.x = enu_pos[0];
    enu_absolute_rtk_deadreckoning->enu_pos.y = enu_pos[1];
    enu_absolute_rtk_deadreckoning->enu_pos.z = enu_pos[2];

    rtk_deadreckoning_status->time_last = enu_vel_time;
    rtk_deadreckoning_status->position_stamp_last = fix_time;
  }
  else
  {
    enu_absolute_rtk_deadreckoning->status.enabled_status = false;
    enu_absolute_rtk_deadreckoning->status.estimate_status = false;
  }
}
