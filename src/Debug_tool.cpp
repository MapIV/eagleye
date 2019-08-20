/*
 * Debug_tool.cpp
 * Debug tool
 * Author Sekino
 * Ver 1.00 2019/3/28
 */

#include "ros/ros.h"
#include "imu_gnss_localizer/Debug_tool.h"
#include "imu_gnss_localizer/Distance.h"
#include "imu_gnss_localizer/Heading.h"
#include "imu_gnss_localizer/PositionDis_raw.h"
#include "imu_gnss_localizer/PositionDis.h"
#include "imu_gnss_localizer/RTKLIB.h"
#include "imu_gnss_localizer/UsrVel_enu.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include "imu_gnss_localizer/YawrateOffset.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

ros::Publisher pub;

// rtklib_connecter
int rtklib_GPSTime = 0;
double rtklib_enu_x = 0.0;
double rtklib_enu_y = 0.0;
double rtklib_enu_z = 0.0;
double rtklib_Vel_e = 0.0;
double rtklib_Vel_n = 0.0;
double rtklib_Vel_u = 0.0;
double rtklib_ORG_x = 0.0;
double rtklib_ORG_y = 0.0;
double rtklib_ORG_z = 0.0;
double rtklib_latitude = 0.0;
double rtklib_longitude = 0.0;
double rtklib_altitude = 0.0;
double rtklib_Doppler_Velocity = 0.0;
double rtklib_Doppler_Heading_angle = 0.0;

// RCalc_VelocitySF
double VelocitySF_Correction_Velocity = 0.0;
double VelocitySF_Scale_Factor = 0.0;
bool VelocitySF_flag_Est = 0;
bool VelocitySF_flag_EstRaw = 0;

// RCalc_Heading1st
double Heading1st_Heading_angle = 0.0;
bool Heading1st_flag_Est = 0;
bool Heading1st_flag_EstRaw = 0;

// RCalc_Heading2nd
double Heading2nd_Heading_angle = 0.0;
bool Heading2nd_flag_Est = 0;
bool Heading2nd_flag_EstRaw = 0;

// RCalc_Heading3rd
double Heading3rd_Heading_angle = 0.0;
bool Heading3rd_flag_Est = 0;
bool Heading3rd_flag_EstRaw = 0;

// RCalc_YawrateOffsetStop
double YawrateOffsetStop_YawrateOffset = 0.0;
bool YawrateOffsetStop_flag_Est = 0;
bool YawrateOffsetStop_flag_EstRaw = 0;

// RCalc_YawrateOffset1st
double YawrateOffset1st_YawrateOffset = 0.0;
bool YawrateOffset1st_flag_Est = 0;
bool YawrateOffset1st_flag_EstRaw = 0;

// RCalc_YawrateOffset2nd
double YawrateOffset2nd_YawrateOffset = 0.0;
bool YawrateOffset2nd_flag_Est = 0;
bool YawrateOffset2nd_flag_EstRaw = 0;

// RCalc_Trajectory
double Trajectory_VelE = 0.0;
double Trajectory_VelN = 0.0;
double Trajectory_VelU = 0.0;
int Trajectory_time_stamp = 0;
double Trajectory_x = 0.0;
double Trajectory_y = 0.0;
double Trajectory_z = 0.0;
double Trajectory_Heading_angle = 0.0;

// RCalc_Distance
double Distance_Distance = 0.0;

// RCalc_PositionDis
double PositionDis_enu_x = 0.0;
double PositionDis_enu_y = 0.0;
double PositionDis_enu_z = 0.0;
int PositionDis_time_stamp = 0;
int PositionDis_log_1 = 0;
int PositionDis_log_2 = 0;
int PositionDis_log_3 = 0;
int PositionDis_log_4 = 0;

// RCalc_PositionDis_Int
double PositionDis_Int_enu_x = 0.0;
double PositionDis_Int_enu_y = 0.0;
double PositionDis_Int_enu_z = 0.0;
double PositionDis_Int_latitude = 0.0;
double PositionDis_Int_longitude = 0.0;
double PositionDis_Int_altitude = 0.0;
bool PositionDis_Int_flag_Est = 0;
bool PositionDis_Int_flag_EstRaw = 0;

// CAN_Velocity
double CAN_Velocity = 0.0;

// IMU_rawdata
int IMU_GPSTime = 0;
double IMU_acceleration_x = 0.0;
double IMU_acceleration_y = 0.0;
double IMU_acceleration_z = 0.0;
double IMU_angular_velocity_x = 0.0;
double IMU_angular_velocity_y = 0.0;
double IMU_angular_velocity_z = 0.0;

//計算用
double velN = 0.0;
double velE = 0.0;
double velU = 0.0;

imu_gnss_localizer::Debug_tool p_msg;

void receive_rtklib(const imu_gnss_localizer::RTKLIB::ConstPtr& msg)
{
  rtklib_GPSTime = msg->GPSTime;
  rtklib_enu_x = msg->enu_x;
  rtklib_enu_y = msg->enu_y;
  rtklib_enu_z = msg->enu_z;
  rtklib_Vel_e = msg->Vel_e;
  rtklib_Vel_n = msg->Vel_n;
  rtklib_Vel_u = msg->Vel_u;
  rtklib_ORG_x = msg->ORG_x;
  rtklib_ORG_y = msg->ORG_y;
  rtklib_ORG_z = msg->ORG_z;

  velE = msg->Vel_e;                                                              // unit [m/s]
  velN = msg->Vel_n;                                                              // unit [m/s]
  velU = msg->Vel_u;                                                              // unit [m/s]
  rtklib_Doppler_Velocity = sqrt((velE * velE) + (velN * velN) + (velU * velU));  // unit [m/s]
  rtklib_Doppler_Heading_angle = atan2(velE, velN);                               // unit [rad]
}

void receive_rtklib_fix(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  rtklib_latitude = msg->latitude;
  rtklib_longitude = msg->longitude;
  rtklib_altitude = msg->altitude;
}

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg)
{
  VelocitySF_Correction_Velocity = msg->Correction_Velocity;
  VelocitySF_Scale_Factor = msg->Scale_Factor;
  VelocitySF_flag_Est = msg->flag_Est;
  VelocitySF_flag_EstRaw = msg->flag_EstRaw;
}

void receive_Heading1st(const imu_gnss_localizer::Heading::ConstPtr& msg)
{
  Heading1st_Heading_angle = msg->Heading_angle;
  Heading1st_flag_Est = msg->flag_Est;
  Heading1st_flag_EstRaw = msg->flag_EstRaw;
}

void receive_Heading2nd(const imu_gnss_localizer::Heading::ConstPtr& msg)
{
  Heading2nd_Heading_angle = msg->Heading_angle;
  Heading2nd_flag_Est = msg->flag_Est;
  Heading2nd_flag_EstRaw = msg->flag_EstRaw;
}

void receive_Heading3rd(const imu_gnss_localizer::Heading::ConstPtr& msg)
{
  Heading3rd_Heading_angle = msg->Heading_angle;
  Heading3rd_flag_Est = msg->flag_Est;
  Heading3rd_flag_EstRaw = msg->flag_EstRaw;
}

void receive_YawrateOffsetStop(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffsetStop_YawrateOffset = msg->YawrateOffset;
  YawrateOffsetStop_flag_Est = msg->flag_Est;
  YawrateOffsetStop_flag_EstRaw = msg->flag_EstRaw;
}

void receive_YawrateOffset1st(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset1st_YawrateOffset = msg->YawrateOffset;
  YawrateOffset1st_flag_Est = msg->flag_Est;
  YawrateOffset1st_flag_EstRaw = msg->flag_EstRaw;
}

void receive_YawrateOffset2nd(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset2nd_YawrateOffset = msg->YawrateOffset;
  YawrateOffset2nd_flag_Est = msg->flag_Est;
  YawrateOffset2nd_flag_EstRaw = msg->flag_EstRaw;
}

void receive_UsrVel_enu(const imu_gnss_localizer::UsrVel_enu::ConstPtr& msg)
{
  Trajectory_VelE = msg->VelE;
  Trajectory_VelN = msg->VelN;
  Trajectory_VelU = msg->VelU;
  Trajectory_time_stamp = msg->time_stamp;
}

void receive_Trajectory(const geometry_msgs::Pose::ConstPtr& msg)
{
  Trajectory_x = msg->position.x;
  Trajectory_y = msg->position.y;
  Trajectory_z = msg->position.z;
  Trajectory_Heading_angle = msg->orientation.z;
}

void receive_Distance(const imu_gnss_localizer::Distance::ConstPtr& msg)
{
  Distance_Distance = msg->Distance;
}

void receive_PositionDis(const imu_gnss_localizer::PositionDis_raw::ConstPtr& msg)
{
  PositionDis_enu_x = msg->enu_x;
  PositionDis_enu_y = msg->enu_y;
  PositionDis_enu_z = msg->enu_z;
  PositionDis_time_stamp = msg->time_stamp;
  PositionDis_log_1 = msg->log_1;
  PositionDis_log_2 = msg->log_2;
  PositionDis_log_3 = msg->log_3;
  PositionDis_log_4 = msg->log_4;
}

void receive_PositionDis_Int(const imu_gnss_localizer::PositionDis::ConstPtr& msg)
{
  PositionDis_Int_enu_x = msg->enu_x;
  PositionDis_Int_enu_y = msg->enu_y;
  PositionDis_Int_enu_z = msg->enu_z;
  PositionDis_Int_latitude = msg->latitude;
  PositionDis_Int_longitude = msg->longitude;
  PositionDis_Int_altitude = msg->altitude;
  PositionDis_Int_flag_Est = msg->flag_Est;
  PositionDis_Int_flag_EstRaw = msg->flag_EstRaw;
}

void receive_Velocity(const geometry_msgs::Twist::ConstPtr& msg)
{
  CAN_Velocity = msg->linear.x;
}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  IMU_acceleration_x = msg->linear_acceleration.x;
  IMU_acceleration_y = msg->linear_acceleration.y;
  IMU_acceleration_z = msg->linear_acceleration.z;
  IMU_angular_velocity_x = msg->angular_velocity.x;
  IMU_angular_velocity_y = msg->angular_velocity.y;
  IMU_angular_velocity_z = msg->angular_velocity.z;

  // rtklib_connecter
  p_msg.rtklib_GPSTime = rtklib_GPSTime;
  p_msg.rtklib_enu_x = rtklib_enu_x;
  p_msg.rtklib_enu_y = rtklib_enu_y;
  p_msg.rtklib_enu_z = rtklib_enu_z;
  p_msg.rtklib_Vel_e = rtklib_Vel_e;
  p_msg.rtklib_Vel_n = rtklib_Vel_n;
  p_msg.rtklib_Vel_u = rtklib_Vel_u;
  p_msg.rtklib_ORG_x = rtklib_ORG_x;
  p_msg.rtklib_ORG_y = rtklib_ORG_y;
  p_msg.rtklib_ORG_z = rtklib_ORG_z;
  p_msg.rtklib_latitude = rtklib_latitude;
  p_msg.rtklib_longitude = rtklib_longitude;
  p_msg.rtklib_altitude = rtklib_altitude;
  p_msg.rtklib_Doppler_Velocity = rtklib_Doppler_Velocity;
  p_msg.rtklib_Doppler_Heading_angle = rtklib_Doppler_Heading_angle;

  // RCalc_VelocitySF
  p_msg.VelocitySF_Correction_Velocity = VelocitySF_Correction_Velocity;
  p_msg.VelocitySF_Scale_Factor = VelocitySF_Scale_Factor;
  p_msg.VelocitySF_flag_Est = VelocitySF_flag_Est;
  p_msg.VelocitySF_flag_EstRaw = VelocitySF_flag_EstRaw;
  VelocitySF_flag_EstRaw = false;

  // RCalc_Heading1st
  p_msg.Heading1st_Heading_angle = Heading1st_Heading_angle;
  p_msg.Heading1st_flag_Est = Heading1st_flag_Est;
  p_msg.Heading1st_flag_EstRaw = Heading1st_flag_EstRaw;
  Heading1st_flag_EstRaw = false;

  // RCalc_Heading2nd
  p_msg.Heading2nd_Heading_angle = Heading2nd_Heading_angle;
  p_msg.Heading2nd_flag_Est = Heading2nd_flag_Est;
  p_msg.Heading2nd_flag_EstRaw = Heading2nd_flag_EstRaw;
  Heading2nd_flag_EstRaw = false;

  // RCalc_Heading3rd
  p_msg.Heading3rd_Heading_angle = Heading3rd_Heading_angle;
  p_msg.Heading3rd_flag_Est = Heading3rd_flag_Est;
  p_msg.Heading3rd_flag_EstRaw = Heading3rd_flag_EstRaw;
  Heading3rd_flag_EstRaw = false;

  // RCalc_YawrateOffsetStop
  p_msg.YawrateOffsetStop_YawrateOffset = YawrateOffsetStop_YawrateOffset;
  p_msg.YawrateOffsetStop_flag_Est = YawrateOffsetStop_flag_Est;
  p_msg.YawrateOffsetStop_flag_EstRaw = YawrateOffsetStop_flag_EstRaw;
  YawrateOffsetStop_flag_EstRaw = false;

  // RCalc_YawrateOffset1st
  p_msg.YawrateOffset1st_YawrateOffset = YawrateOffset1st_YawrateOffset;
  p_msg.YawrateOffset1st_flag_Est = YawrateOffset1st_flag_Est;
  p_msg.YawrateOffset1st_flag_EstRaw = YawrateOffset1st_flag_EstRaw;
  YawrateOffset1st_flag_EstRaw = false;

  // RCalc_YawrateOffset2nd
  p_msg.YawrateOffset2nd_YawrateOffset = YawrateOffset2nd_YawrateOffset;
  p_msg.YawrateOffset2nd_flag_Est = YawrateOffset2nd_flag_Est;
  p_msg.YawrateOffset2nd_flag_EstRaw = YawrateOffset2nd_flag_EstRaw;
  YawrateOffset2nd_flag_EstRaw = false;

  // RCalc_Trajectory
  p_msg.Trajectory_VelE = Trajectory_VelE;
  p_msg.Trajectory_VelN = Trajectory_VelN;
  p_msg.Trajectory_VelU = Trajectory_VelU;
  p_msg.Trajectory_time_stamp = Trajectory_time_stamp;
  p_msg.Trajectory_x = Trajectory_x;
  p_msg.Trajectory_y = Trajectory_y;
  p_msg.Trajectory_z = Trajectory_z;
  p_msg.Trajectory_Heading_angle = Trajectory_Heading_angle;

  // RCalc_Distance
  p_msg.Distance_Distance = Distance_Distance;

  // RCalc_PositionDis
  p_msg.PositionDis_enu_x = PositionDis_enu_x;
  p_msg.PositionDis_enu_y = PositionDis_enu_y;
  p_msg.PositionDis_enu_z = PositionDis_enu_z;
  p_msg.PositionDis_time_stamp = PositionDis_time_stamp;

  // RCalc_PositionDis_Int
  p_msg.PositionDis_Int_enu_x = PositionDis_Int_enu_x;
  p_msg.PositionDis_Int_enu_y = PositionDis_Int_enu_y;
  p_msg.PositionDis_Int_enu_z = PositionDis_Int_enu_z;
  p_msg.PositionDis_Int_flag_Est = PositionDis_Int_flag_Est;
  p_msg.PositionDis_Int_flag_EstRaw = PositionDis_Int_flag_EstRaw;
  p_msg.PositionDis_Int_latitude = PositionDis_Int_latitude;
  p_msg.PositionDis_Int_longitude = PositionDis_Int_longitude;
  p_msg.PositionDis_Int_altitude = PositionDis_Int_altitude;
  PositionDis_Int_flag_EstRaw = false;

  // CAN_Velocity
  p_msg.CAN_Velocity = CAN_Velocity;

  // IMU_rawdata
  p_msg.IMU_GPSTime = IMU_GPSTime;
  p_msg.IMU_acceleration_x = IMU_acceleration_x;
  p_msg.IMU_acceleration_y = IMU_acceleration_y;
  p_msg.IMU_acceleration_z = IMU_acceleration_z;
  p_msg.IMU_angular_velocity_x = IMU_angular_velocity_x;
  p_msg.IMU_angular_velocity_y = IMU_angular_velocity_y;
  p_msg.IMU_angular_velocity_z = IMU_angular_velocity_z;

  // RCalc_PositionDis_Debug
  p_msg.PositionDis_log_1 = PositionDis_log_1;
  p_msg.PositionDis_log_2 = PositionDis_log_2;
  p_msg.PositionDis_log_3 = PositionDis_log_3;
  p_msg.PositionDis_log_4 = PositionDis_log_4;
  PositionDis_log_1 = 0;
  PositionDis_log_2 = 0;
  PositionDis_log_3 = 0;

  pub.publish(p_msg);

  IMU_GPSTime += 1000 / 50;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Debug_tool");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/RTKLIB", 1000, receive_rtklib);
  ros::Subscriber sub2 = n.subscribe("/fix", 1000, receive_rtklib_fix);
  ros::Subscriber sub3 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub4 = n.subscribe("/imu_gnss_localizer/Heading1st_Int", 1000, receive_Heading1st);
  ros::Subscriber sub5 = n.subscribe("/imu_gnss_localizer/Heading2nd_Int", 1000, receive_Heading2nd);
  ros::Subscriber sub6 = n.subscribe("/imu_gnss_localizer/Heading3rd_Int", 1000, receive_Heading3rd);
  ros::Subscriber sub7 = n.subscribe("/imu_gnss_localizer/YawrateOffsetStop", 1000, receive_YawrateOffsetStop);
  ros::Subscriber sub8 = n.subscribe("/imu_gnss_localizer/YawrateOffset1st", 1000, receive_YawrateOffset1st);
  ros::Subscriber sub9 = n.subscribe("/imu_gnss_localizer/YawrateOffset2nd", 1000, receive_YawrateOffset2nd);
  ros::Subscriber sub10 = n.subscribe("/imu_gnss_localizer/UsrVel_enu", 1000, receive_UsrVel_enu);
  ros::Subscriber sub11 = n.subscribe("/imu_gnss_localizer/Trajectory", 1000, receive_Trajectory);
  ros::Subscriber sub12 = n.subscribe("/imu_gnss_localizer/Distance", 1000, receive_Distance);
  ros::Subscriber sub13 = n.subscribe("/imu_gnss_localizer/PositionDis", 1000, receive_PositionDis);
  ros::Subscriber sub14 = n.subscribe("/imu_gnss_localizer/PositionDis_Int", 1000, receive_PositionDis_Int);
  ros::Subscriber sub15 = n.subscribe("/Vehicle/Velocity", 1000, receive_Velocity);
  ros::Subscriber sub16 = n.subscribe("/imu/data_raw", 1000, receive_Imu);

  pub = n.advertise<imu_gnss_localizer::Debug_tool>("/imu_gnss_localizer/Debug_tool", 1000);

  ros::spin();

  return 0;
}
