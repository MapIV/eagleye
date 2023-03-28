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

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

void rtk_deadreckoning_estimate_(geometry_msgs::Vector3Stamped enu_vel, nmea_msgs::Gpgga gga,  eagleye_msgs::Heading heading,
  RtkDeadreckoningParameter rtk_deadreckoning_parameter, RtkDeadreckoningStatus* rtk_deadreckoning_status,
  eagleye_msgs::Position* enu_absolute_rtk_deadreckoning,sensor_msgs::NavSatFix* eagleye_fix)
{

  double enu_pos[3],enu_rtk[3];
  double ecef_base_pos[3];
  double ecef_rtk[3];
  double llh_pos[3],llh_rtk[3];

  if(rtk_deadreckoning_status->position_estimate_start_status && heading.status.enabled_status)
  {
    ecef_base_pos[0] = enu_absolute_rtk_deadreckoning->ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_rtk_deadreckoning->ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_rtk_deadreckoning->ecef_base_pos.z;

    llh_rtk[0] = gga.lat *M_PI/180;
    llh_rtk[1] = gga.lon *M_PI/180;
    llh_rtk[2] = gga.alt + gga.undulation;

    llh2xyz(llh_rtk,ecef_rtk);
    xyz2enu(ecef_rtk,ecef_base_pos,enu_rtk);

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = enu_rtk[0];
    pose.pose.position.y = enu_rtk[1];
    pose.pose.position.z = enu_rtk[2];

    heading.heading_angle = fmod(heading.heading_angle,2*M_PI);
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, (90* M_PI / 180)-heading.heading_angle);
    transform.setRotation(q);

    tf::Transform transform2, transform3;
    tf::Quaternion q2(rtk_deadreckoning_parameter.tf_gnss_rotation_x,rtk_deadreckoning_parameter.tf_gnss_rotation_y,
      rtk_deadreckoning_parameter.tf_gnss_rotation_z,rtk_deadreckoning_parameter.tf_gnss_rotation_w);
    transform2.setOrigin(tf::Vector3(rtk_deadreckoning_parameter.tf_gnss_translation_x, rtk_deadreckoning_parameter.tf_gnss_translation_y,
      rtk_deadreckoning_parameter.tf_gnss_translation_z));
    transform2.setRotation(q2);
    transform3 = transform * transform2.inverse();
    tf::Vector3 tmp_pos;
    tmp_pos = transform3.getOrigin();

    enu_rtk[0] = tmp_pos.getX();
    enu_rtk[1] = tmp_pos.getY();
    enu_rtk[2] = tmp_pos.getZ();

    if (rtk_deadreckoning_status->position_stamp_last != gga.header.stamp.toSec() && gga.gps_qual == 4)
    {
      rtk_deadreckoning_status->provisional_enu_pos_x = enu_rtk[0];
      rtk_deadreckoning_status->provisional_enu_pos_y = enu_rtk[1];
      rtk_deadreckoning_status->provisional_enu_pos_z = enu_rtk[2];
      enu_absolute_rtk_deadreckoning->status.enabled_status = true;
      enu_absolute_rtk_deadreckoning->status.estimate_status = true;
    }
    else if(rtk_deadreckoning_status->time_last != 0 && sqrt((enu_vel.vector.x * enu_vel.vector.x) + (enu_vel.vector.y * enu_vel.vector.y) + (enu_vel.vector.z * enu_vel.vector.z)) > rtk_deadreckoning_parameter.stop_judgment_threshold)
    {
      rtk_deadreckoning_status->provisional_enu_pos_x = enu_absolute_rtk_deadreckoning->enu_pos.x + enu_vel.vector.x * (enu_vel.header.stamp.toSec() - rtk_deadreckoning_status->time_last);
      rtk_deadreckoning_status->provisional_enu_pos_y = enu_absolute_rtk_deadreckoning->enu_pos.y + enu_vel.vector.y * (enu_vel.header.stamp.toSec() - rtk_deadreckoning_status->time_last);
      rtk_deadreckoning_status->provisional_enu_pos_z = enu_absolute_rtk_deadreckoning->enu_pos.z + enu_vel.vector.z * (enu_vel.header.stamp.toSec() - rtk_deadreckoning_status->time_last);
      enu_absolute_rtk_deadreckoning->status.enabled_status = true;
      enu_absolute_rtk_deadreckoning->status.estimate_status = false;
    }
    else if(!enu_absolute_rtk_deadreckoning->status.enabled_status)
    {
      return;
    }

    enu_pos[0] = rtk_deadreckoning_status->provisional_enu_pos_x;
    enu_pos[1] = rtk_deadreckoning_status->provisional_enu_pos_y;
    enu_pos[2] = rtk_deadreckoning_status->provisional_enu_pos_z;

    enu2llh(enu_pos, ecef_base_pos, llh_pos);

    double rtk_fix_STD = rtk_deadreckoning_parameter.rtk_fix_STD;
    Eigen::MatrixXd init_covariance;
    init_covariance = Eigen::MatrixXd::Zero(6, 6);
    init_covariance(0,0) = rtk_fix_STD * rtk_fix_STD;
    init_covariance(1,1) = rtk_fix_STD * rtk_fix_STD;
    init_covariance(2,2) = rtk_fix_STD * rtk_fix_STD;
    init_covariance(5,5) = heading.variance;

    double proc_noise = rtk_deadreckoning_parameter.proc_noise;
    Eigen::MatrixXd proc_covariance;
    proc_covariance = Eigen::MatrixXd::Zero(6, 6);
    proc_covariance(0,0) = proc_noise * proc_noise;
    proc_covariance(1,1) = proc_noise * proc_noise;
    proc_covariance(2,2) = proc_noise * proc_noise;

    Eigen::MatrixXd position_covariance;
    position_covariance = Eigen::MatrixXd::Zero(6, 6);


    if(enu_absolute_rtk_deadreckoning->status.estimate_status)
    {
      position_covariance = init_covariance;
      rtk_deadreckoning_status->position_covariance_last = position_covariance;
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
      jacobian(0,5) = enu_vel.vector.y*(enu_vel.header.stamp.toSec() - rtk_deadreckoning_status->time_last);
      jacobian(1,5) = -enu_vel.vector.x*(enu_vel.header.stamp.toSec() - rtk_deadreckoning_status->time_last);

      // MEMO: Jacobean not included
      // position_covariance = rtk_deadreckoning_status->position_covariance_last + proc_covariance;

      // MEMO: Jacobean not included
      position_covariance = jacobian * rtk_deadreckoning_status->position_covariance_last * (jacobian.transpose())   + proc_covariance;

      rtk_deadreckoning_status->position_covariance_last = position_covariance;
    }

    eagleye_fix->latitude = llh_pos[0] * 180/M_PI;
    eagleye_fix->longitude = llh_pos[1] * 180/M_PI;
    eagleye_fix->altitude = llh_pos[2];
    eagleye_fix->position_covariance[0] = position_covariance(0,0); // [m^2]
    eagleye_fix->position_covariance[4] = position_covariance(1,1); // [m^2]
    eagleye_fix->position_covariance[8] = position_covariance(2,2); // [m^2]

    enu_absolute_rtk_deadreckoning->enu_pos.x = enu_pos[0];
    enu_absolute_rtk_deadreckoning->enu_pos.y = enu_pos[1];
    enu_absolute_rtk_deadreckoning->enu_pos.z = enu_pos[2];
    enu_absolute_rtk_deadreckoning->covariance[0] = position_covariance(0,0); // [m^2]
    enu_absolute_rtk_deadreckoning->covariance[4] = position_covariance(1,1); // [m^2]
    enu_absolute_rtk_deadreckoning->covariance[8] = position_covariance(2,2); // [m^2]

    rtk_deadreckoning_status->time_last = enu_vel.header.stamp.toSec();
    rtk_deadreckoning_status->position_stamp_last = gga.header.stamp.toSec();
  }
  else
  {
    enu_absolute_rtk_deadreckoning->status.enabled_status = false;
    enu_absolute_rtk_deadreckoning->status.estimate_status = false;
  }
}

void rtk_deadreckoning_estimate(rtklib_msgs::RtklibNav rtklib_nav,geometry_msgs::Vector3Stamped enu_vel, nmea_msgs::Gpgga gga,  eagleye_msgs::Heading heading, RtkDeadreckoningParameter rtk_deadreckoning_parameter, RtkDeadreckoningStatus* rtk_deadreckoning_status, eagleye_msgs::Position* enu_absolute_rtk_deadreckoning,sensor_msgs::NavSatFix* eagleye_fix)
{
  if(rtk_deadreckoning_parameter.use_ecef_base_position)
  {
    enu_absolute_rtk_deadreckoning->ecef_base_pos.x = rtk_deadreckoning_parameter.ecef_base_pos_x;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.y = rtk_deadreckoning_parameter.ecef_base_pos_y;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.z = rtk_deadreckoning_parameter.ecef_base_pos_z;
    rtk_deadreckoning_status->ecef_base_pos_status = true;
    rtk_deadreckoning_status->position_estimate_start_status = true;
  }
  else if(!rtk_deadreckoning_status->ecef_base_pos_status && rtklib_nav.header.stamp.toSec() != 0)
  {
    enu_absolute_rtk_deadreckoning->ecef_base_pos.x = rtklib_nav.ecef_pos.x;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.y = rtklib_nav.ecef_pos.y;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.z = rtklib_nav.ecef_pos.z;
    rtk_deadreckoning_status->ecef_base_pos_status = true;
    rtk_deadreckoning_status->position_estimate_start_status = true;
  }

  rtk_deadreckoning_estimate_(enu_vel, gga, heading, rtk_deadreckoning_parameter, rtk_deadreckoning_status, enu_absolute_rtk_deadreckoning, eagleye_fix);
}

void rtk_deadreckoning_estimate(geometry_msgs::Vector3Stamped enu_vel, nmea_msgs::Gpgga gga,  eagleye_msgs::Heading heading, RtkDeadreckoningParameter rtk_deadreckoning_parameter, RtkDeadreckoningStatus* rtk_deadreckoning_status, eagleye_msgs::Position* enu_absolute_rtk_deadreckoning,sensor_msgs::NavSatFix* eagleye_fix)
{
  double ecef_pos[3];
  double llh_pos[3];

  if(rtk_deadreckoning_parameter.use_ecef_base_position)
  {
    enu_absolute_rtk_deadreckoning->ecef_base_pos.x = rtk_deadreckoning_parameter.ecef_base_pos_x;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.y = rtk_deadreckoning_parameter.ecef_base_pos_y;
    enu_absolute_rtk_deadreckoning->ecef_base_pos.z = rtk_deadreckoning_parameter.ecef_base_pos_z;
    rtk_deadreckoning_status->ecef_base_pos_status = true;
    rtk_deadreckoning_status->position_estimate_start_status = true;
  }
  else if(!rtk_deadreckoning_status->ecef_base_pos_status && gga.header.stamp.toSec() != 0)
  {

    llh_pos[0] = gga.lat *M_PI/180;
    llh_pos[1] = gga.lon *M_PI/180;
    llh_pos[2] = gga.alt + gga.undulation;

    llh2xyz(llh_pos,ecef_pos);

    enu_absolute_rtk_deadreckoning->ecef_base_pos.x = ecef_pos[0];
    enu_absolute_rtk_deadreckoning->ecef_base_pos.y = ecef_pos[1];
    enu_absolute_rtk_deadreckoning->ecef_base_pos.z = ecef_pos[2];
    rtk_deadreckoning_status->ecef_base_pos_status = true;
    rtk_deadreckoning_status->position_estimate_start_status = true;
  }

  rtk_deadreckoning_estimate_(enu_vel, gga, heading, rtk_deadreckoning_parameter, rtk_deadreckoning_status, enu_absolute_rtk_deadreckoning, eagleye_fix);
}
