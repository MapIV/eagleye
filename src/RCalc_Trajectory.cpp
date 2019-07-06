/*
 * RCalc_Trajectory.cpp
 * Trajectory estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include "imu_gnss_localizer/Heading.h"
#include "imu_gnss_localizer/YawrateOffset.h"
#include "imu_gnss_localizer/UsrVel_enu.h"

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

// default parameter
bool reverse_imu = false;

bool flag_Est, flag_SF, flag_YOS, flag_YawrateOffsetEst;
int count = 0;
int T_count = 0;
double TH_VEL_STOP = 0.01;
double IMUTime;
double IMUfrequency = 50;  // IMU Hz
double IMUperiod = 1 / IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
double Est_velE = 0.0;
double Est_velN = 0.0;
double Est_velU = 0.0;
double Trajectory_x = 0.0;
double Trajectory_y = 0.0;
double Trajectory_z = 0.0;
double Trajectory_x_Last = 0.0;
double Trajectory_y_Last = 0.0;
double Trajectory_z_Last = 0.0;
double pVelocity = 0.0;
double pHeading = 0.0;
double YawrateOffset = 0.0;
double YawrateOffset_Stop = 0.0;
double Yawrate = 0.0;

geometry_msgs::Pose p1_msg;
imu_gnss_localizer::UsrVel_enu p2_msg;
geometry_msgs::TwistStamped p3_msg;

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg)
{
  pVelocity = msg->Correction_Velocity;
  flag_SF = msg->flag_Est;
}

void receive_Heading3rd(const imu_gnss_localizer::Heading::ConstPtr& msg)
{
  pHeading = msg->Heading_angle;
  flag_Est = msg->flag_Est;
}

void receive_YawrateOffsetStop(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset_Stop = msg->YawrateOffset;
  flag_YOS = msg->flag_Est;
}

void receive_YawrateOffset2nd(const imu_gnss_localizer::YawrateOffset::ConstPtr& msg)
{
  YawrateOffset = msg->YawrateOffset;
  flag_YawrateOffsetEst = msg->flag_Est;
}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  ++count;
  IMUTime = IMUperiod * count;
  ROSTime = ros::Time::now().toSec();
  Time = ROSTime;  // IMUTime or ROSTime
  // ROS_INFO("Time = %lf" , Time);

  if (reverse_imu == false)
  {
    if (pVelocity > TH_VEL_STOP && flag_YawrateOffsetEst == true)
    {
      Yawrate = msg->angular_velocity.z + YawrateOffset;
    }
    else
    {
      Yawrate = msg->angular_velocity.z + YawrateOffset_Stop;
    }
  }
  else if (pVelocity > TH_VEL_STOP && flag_YawrateOffsetEst == true)
  {
    if (pVelocity > TH_VEL_STOP)
    {
      Yawrate = (-1 * msg->angular_velocity.z) + YawrateOffset;
    }
    else
    {
      Yawrate = (-1 * msg->angular_velocity.z) + YawrateOffset_Stop;
    }
  }

  p3_msg.header.stamp = ros::Time::now();
  p3_msg.header.frame_id = "/base_link";
  p3_msg.twist.linear.x = pVelocity;
  p3_msg.twist.angular.z = Yawrate;
  pub3.publish(p3_msg);

  if (T_count == 0 && flag_SF == true && flag_Est == true)
  {
    T_count = 1;
  }
  else if (T_count == 1)
  {
    T_count = 2;
  }

  if (T_count == 2)
  {
    Est_velE = sin(pHeading) * pVelocity;
    Est_velN = cos(pHeading) * pVelocity;
  }

  p2_msg.VelE = Est_velE;
  p2_msg.VelN = Est_velN;
  p2_msg.VelU = Est_velU;
  p2_msg.time_stamp = ros::Time::now().toSec();
  pub2.publish(p2_msg);

  if (T_count == 2 && pVelocity > 0 && count > 1)
  {
    Trajectory_x = Trajectory_x_Last + Est_velE * (Time - Time_Last);
    Trajectory_y = Trajectory_y_Last + Est_velN * (Time - Time_Last);
  }
  else
  {
    Trajectory_x = Trajectory_x_Last;
    Trajectory_y = Trajectory_y_Last;
  }

  p1_msg.position.x = Trajectory_x;
  p1_msg.position.y = Trajectory_y;
  p1_msg.orientation.z = pHeading;
  pub1.publish(p1_msg);

  Time_Last = Time;
  Trajectory_x_Last = Trajectory_x;
  Trajectory_y_Last = Trajectory_y;
  Trajectory_z_Last = Trajectory_z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RCalc_Trajectory");

  ros::NodeHandle n;
  n.getParam("/imu_gnss_localizer/reverse_imu", reverse_imu);

  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/Heading3rd_Int", 1000, receive_Heading3rd);
  ros::Subscriber sub3 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  ros::Subscriber sub4 = n.subscribe("/imu_gnss_localizer/YawrateOffsetStop", 1000, receive_YawrateOffsetStop);
  ros::Subscriber sub5 = n.subscribe("/imu_gnss_localizer/YawrateOffset2nd", 1000, receive_YawrateOffset2nd);
  pub1 = n.advertise<geometry_msgs::Pose>("/imu_gnss_localizer/Trajectory", 1000);
  pub2 = n.advertise<imu_gnss_localizer::UsrVel_enu>("/imu_gnss_localizer/UsrVel_enu", 1000);
  pub3 = n.advertise<geometry_msgs::TwistStamped>("/imu_gnss_localizer_twist", 1000);

  ros::spin();

  return 0;
}
