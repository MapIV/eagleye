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
 * rtk_dead_reckoning.cpp
 * Author MapIV Sekino
 */

#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

void rtk_dead_reckoning_estimate_(geometry_msgs::msg::Vector3Stamped enu_vel, nmea_msgs::msg::Gpgga gga,
  eagleye_msgs::msg::Heading heading, RtkDeadreckoningParameter rtk_dead_reckoning_parameter, RtkDeadreckoningStatus* rtk_dead_reckoning_status,
  eagleye_msgs::msg::Position* enu_absolute_rtk_dead_reckoning,sensor_msgs::msg::NavSatFix* eagleye_fix)
{

  double enu_pos[3],enu_rtk[3];
  double ecef_base_pos[3];
  double ecef_rtk[3];
  double llh_pos[3],llh_rtk[3];

  rclcpp::Time ros_clock(gga.header.stamp);
  rclcpp::Time ros_clock2(enu_vel.header.stamp);

  auto gga_time = ros_clock.seconds();
  auto enu_vel_time = ros_clock2.seconds();

  if(rtk_dead_reckoning_status->position_estimate_start_status && heading.status.enabled_status)
  {
    ecef_base_pos[0] = enu_absolute_rtk_dead_reckoning->ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_rtk_dead_reckoning->ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_rtk_dead_reckoning->ecef_base_pos.z;

    llh_rtk[0] = gga.lat *M_PI/180;
    llh_rtk[1] = gga.lon *M_PI/180;
    llh_rtk[2] = gga.alt + gga.undulation;

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
    tf2::Quaternion q2(rtk_dead_reckoning_parameter.tf_gnss_rotation_x,rtk_dead_reckoning_parameter.tf_gnss_rotation_y,rtk_dead_reckoning_parameter.tf_gnss_rotation_z,rtk_dead_reckoning_parameter.tf_gnss_rotation_w);
    transform2.setOrigin(transform*tf2::Vector3(-rtk_dead_reckoning_parameter.tf_gnss_translation_x, -rtk_dead_reckoning_parameter.tf_gnss_translation_y,-rtk_dead_reckoning_parameter.tf_gnss_translation_z));
    transform2.setRotation(transform*q2);

    tf2::Vector3 tmp_pos;
    tmp_pos = transform2.getOrigin();

    enu_rtk[0] = tmp_pos.getX();
    enu_rtk[1] = tmp_pos.getY();
    enu_rtk[2] = tmp_pos.getZ();

    if (rtk_dead_reckoning_status->position_stamp_last != gga_time && gga.gps_qual == 4)
    {
      rtk_dead_reckoning_status->provisional_enu_pos_x = enu_rtk[0];
      rtk_dead_reckoning_status->provisional_enu_pos_y = enu_rtk[1];
      rtk_dead_reckoning_status->provisional_enu_pos_z = enu_rtk[2];
      enu_absolute_rtk_dead_reckoning->status.enabled_status = true;
      enu_absolute_rtk_dead_reckoning->status.estimate_status = true;
    }
    else if(rtk_dead_reckoning_status->time_last != 0 && sqrt((enu_vel.vector.x * enu_vel.vector.x) + (enu_vel.vector.y * enu_vel.vector.y) +
      (enu_vel.vector.z * enu_vel.vector.z)) > rtk_dead_reckoning_parameter.stop_judgment_threshold)
    {
      rtk_dead_reckoning_status->provisional_enu_pos_x = enu_absolute_rtk_dead_reckoning->enu_pos.x + enu_vel.vector.x *
        (enu_vel_time - rtk_dead_reckoning_status->time_last);
      rtk_dead_reckoning_status->provisional_enu_pos_y = enu_absolute_rtk_dead_reckoning->enu_pos.y + enu_vel.vector.y *
        (enu_vel_time - rtk_dead_reckoning_status->time_last);
      rtk_dead_reckoning_status->provisional_enu_pos_z = enu_absolute_rtk_dead_reckoning->enu_pos.z + enu_vel.vector.z *
        (enu_vel_time - rtk_dead_reckoning_status->time_last);
      enu_absolute_rtk_dead_reckoning->status.enabled_status = true;
      enu_absolute_rtk_dead_reckoning->status.estimate_status = false;
    }
    else if(!enu_absolute_rtk_dead_reckoning->status.enabled_status)
    {
      return;
    }

    enu_pos[0] = rtk_dead_reckoning_status->provisional_enu_pos_x;
    enu_pos[1] = rtk_dead_reckoning_status->provisional_enu_pos_y;
    enu_pos[2] = rtk_dead_reckoning_status->provisional_enu_pos_z;

    enu2llh(enu_pos, ecef_base_pos, llh_pos);

    double rtk_fix_STD = rtk_dead_reckoning_parameter.rtk_fix_STD;
    Eigen::MatrixXd init_covariance;
    init_covariance = Eigen::MatrixXd::Zero(6, 6);
    init_covariance(0,0) = rtk_fix_STD * rtk_fix_STD;
    init_covariance(1,1) = rtk_fix_STD * rtk_fix_STD;
    init_covariance(2,2) = rtk_fix_STD * rtk_fix_STD;
    init_covariance(5,5) = heading.variance;

    double proc_noise = rtk_dead_reckoning_parameter.proc_noise;
    Eigen::MatrixXd proc_covariance;
    proc_covariance = Eigen::MatrixXd::Zero(6, 6);
    proc_covariance(0,0) = proc_noise * proc_noise;
    proc_covariance(1,1) = proc_noise * proc_noise;
    proc_covariance(2,2) = proc_noise * proc_noise;

    Eigen::MatrixXd position_covariance;
    position_covariance = Eigen::MatrixXd::Zero(6, 6);


    if(enu_absolute_rtk_dead_reckoning->status.estimate_status)
    {
      position_covariance = init_covariance;
      rtk_dead_reckoning_status->position_covariance_last = position_covariance;
    }
    else
    {
      Eigen::MatrixXd jacobian;
      jacobian = Eigen::MatrixXd::Zero(6, 6);
      jacobian(0,0) = 1;
      jacobian(1,1) = 1;
      jacobian(2,2) = 1;
      jacobian(3,3) = 1;
      jacobian(4,4) = 1;
      jacobian(5,5) = 1;
      jacobian(0,5) = enu_vel.vector.y*(enu_vel_time - rtk_dead_reckoning_status->time_last);
      jacobian(1,5) = -enu_vel.vector.x*(enu_vel_time - rtk_dead_reckoning_status->time_last);

      // MEMO: Jacobean not included
      // position_covariance = rtk_dead_reckoning_status->position_covariance_last + proc_covariance;

      // MEMO: Jacobean not included
      position_covariance = jacobian * rtk_dead_reckoning_status->position_covariance_last * (jacobian.transpose())   + proc_covariance;

      rtk_dead_reckoning_status->position_covariance_last = position_covariance;
    }

    eagleye_fix->latitude = llh_pos[0] * 180/M_PI;
    eagleye_fix->longitude = llh_pos[1] * 180/M_PI;
    eagleye_fix->altitude = llh_pos[2];
    eagleye_fix->position_covariance[0] = position_covariance(0,0); // [m^2]
    eagleye_fix->position_covariance[4] = position_covariance(1,1); // [m^2]
    eagleye_fix->position_covariance[8] = position_covariance(2,2); // [m^2]

    enu_absolute_rtk_dead_reckoning->enu_pos.x = enu_pos[0];
    enu_absolute_rtk_dead_reckoning->enu_pos.y = enu_pos[1];
    enu_absolute_rtk_dead_reckoning->enu_pos.z = enu_pos[2];
    enu_absolute_rtk_dead_reckoning->covariance[0] = position_covariance(0,0); // [m^2]
    enu_absolute_rtk_dead_reckoning->covariance[4] = position_covariance(1,1); // [m^2]
    enu_absolute_rtk_dead_reckoning->covariance[8] = position_covariance(2,2); // [m^2]

    rtk_dead_reckoning_status->time_last = enu_vel_time;
    rtk_dead_reckoning_status->position_stamp_last = gga_time;
  }
  else
  {
    enu_absolute_rtk_dead_reckoning->status.enabled_status = false;
    enu_absolute_rtk_dead_reckoning->status.estimate_status = false;
  }
}

void rtk_dead_reckoning_estimate(rtklib_msgs::msg::RtklibNav rtklib_nav,geometry_msgs::msg::Vector3Stamped enu_vel, nmea_msgs::msg::Gpgga gga,
  eagleye_msgs::msg::Heading heading, RtkDeadreckoningParameter rtk_dead_reckoning_parameter, RtkDeadreckoningStatus* rtk_dead_reckoning_status,
  eagleye_msgs::msg::Position* enu_absolute_rtk_dead_reckoning,sensor_msgs::msg::NavSatFix* eagleye_fix)
{
  rclcpp::Time rtklib_nav_clock(rtklib_nav.header.stamp);
  double rtklib_nav_time = rtklib_nav_clock.seconds();

  if(rtk_dead_reckoning_parameter.use_ecef_base_position)
  {
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.x = rtk_dead_reckoning_parameter.ecef_base_pos_x;
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.y = rtk_dead_reckoning_parameter.ecef_base_pos_y;
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.z = rtk_dead_reckoning_parameter.ecef_base_pos_z;
    rtk_dead_reckoning_status->ecef_base_pos_status = true;
    rtk_dead_reckoning_status->position_estimate_start_status = true;
  }
  else if(!rtk_dead_reckoning_status->ecef_base_pos_status && rtklib_nav_time != 0)
  {
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.x = rtklib_nav.ecef_pos.x;
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.y = rtklib_nav.ecef_pos.y;
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.z = rtklib_nav.ecef_pos.z;
    rtk_dead_reckoning_status->ecef_base_pos_status = true;
    rtk_dead_reckoning_status->position_estimate_start_status = true;
  }

  rtk_dead_reckoning_estimate_(enu_vel, gga, heading, rtk_dead_reckoning_parameter, rtk_dead_reckoning_status, enu_absolute_rtk_dead_reckoning, eagleye_fix);
}

void rtk_dead_reckoning_estimate(geometry_msgs::msg::Vector3Stamped enu_vel, nmea_msgs::msg::Gpgga gga,  eagleye_msgs::msg::Heading heading,
  RtkDeadreckoningParameter rtk_dead_reckoning_parameter, RtkDeadreckoningStatus* rtk_dead_reckoning_status,
  eagleye_msgs::msg::Position* enu_absolute_rtk_dead_reckoning,sensor_msgs::msg::NavSatFix* eagleye_fix)
{
  double ecef_pos[3];
  double llh_pos[3];

  rclcpp::Time gga_clock(gga.header.stamp);
  double gga_time = gga_clock.seconds();

  if(rtk_dead_reckoning_parameter.use_ecef_base_position)
  {
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.x = rtk_dead_reckoning_parameter.ecef_base_pos_x;
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.y = rtk_dead_reckoning_parameter.ecef_base_pos_y;
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.z = rtk_dead_reckoning_parameter.ecef_base_pos_z;
    rtk_dead_reckoning_status->ecef_base_pos_status = true;
    rtk_dead_reckoning_status->position_estimate_start_status = true;
  }
  else if(!rtk_dead_reckoning_status->ecef_base_pos_status && gga_time != 0)
  {

    llh_pos[0] = gga.lat *M_PI/180;
    llh_pos[1] = gga.lon *M_PI/180;
    llh_pos[2] = gga.alt + gga.undulation;

    llh2xyz(llh_pos,ecef_pos);

    enu_absolute_rtk_dead_reckoning->ecef_base_pos.x = ecef_pos[0];
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.y = ecef_pos[1];
    enu_absolute_rtk_dead_reckoning->ecef_base_pos.z = ecef_pos[2];
    rtk_dead_reckoning_status->ecef_base_pos_status = true;
    rtk_dead_reckoning_status->position_estimate_start_status = true;
  }

  rtk_dead_reckoning_estimate_(enu_vel, gga, heading, rtk_dead_reckoning_parameter, rtk_dead_reckoning_status, enu_absolute_rtk_dead_reckoning, eagleye_fix);
}
