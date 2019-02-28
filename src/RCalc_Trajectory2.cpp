/*
 * RCalc_Trajectory.cpp
 * Trajectory estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/VelocitySF.h"
#include "imu_gnss_localizer/Heading.h"
#include "imu_gnss_localizer/YawrateOffset.h"
#include "imu_gnss_localizer/UsrVel_enu.h"
#include "imu_gnss_localizer/Position.h"
#include <boost/circular_buffer.hpp>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

bool flag_Est, flag_SF, flag_UsrPos_Est, flag_UsrPos_Start;
int count = 0;
int T_count = 0;
int Est_index = 0;
int UsrPos_index = 0;
int UsrPos_index_Last = 0;
int BUFNUM = 0;
double IMUTime;
double IMUfrequency = 100; //IMU Hz
double IMUperiod = 1/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
float BUFNUM_MAX = 100;
float Est_velE = 0.0;
float Est_velN = 0.0;
float Est_velU = 0.0;
float Trajectory_x = 0.0;
float Trajectory_y = 0.0;
float Trajectory_z = 0.0;
float tUsrPos_enu_x =0.0;
float tUsrPos_enu_y =0.0;
float tUsrPos_enu_z =0.0;
float Trajectory_x_Last = 0.0;
float Trajectory_y_Last = 0.0;
float Trajectory_z_Last = 0.0;
float UsrPos_Est_enu_x = 0.0;
float UsrPos_Est_enu_y = 0.0;
float UsrPos_Est_enu_z = 0.0;
float UsrPos_Est_enu_x_Last = 0.0;
float UsrPos_Est_enu_y_Last = 0.0;
float UsrPos_Est_enu_z_Last = 0.0;
float diff_x = 0.0;
float diff_y = 0.0;
float diff_z = 0.0;
float pVelocity = 0.0;
float pHeading = 0.0;

geometry_msgs::Pose p1_msg;
geometry_msgs::Pose p2_msg;
imu_gnss_localizer::UsrVel_enu p3_msg;

boost::circular_buffer<float> pUsrPos_Est_enu_x(BUFNUM_MAX);
boost::circular_buffer<float> pUsrPos_Est_enu_y(BUFNUM_MAX);
boost::circular_buffer<float> pUsrPos_Est_enu_z(BUFNUM_MAX);
boost::circular_buffer<float> pTrajectory_x(BUFNUM_MAX);
boost::circular_buffer<float> pTrajectory_y(BUFNUM_MAX);
boost::circular_buffer<float> pTrajectory_z(BUFNUM_MAX);

void receive_VelocitySF(const imu_gnss_localizer::VelocitySF::ConstPtr& msg){

  pVelocity = msg->Correction_Velocity;
  flag_SF = msg->flag_Est;

}

void receive_Heading3rd(const imu_gnss_localizer::Heading::ConstPtr& msg){

  pHeading = msg->Heading_angle;
  flag_Est = msg->flag_Est;

}

void receive_PositionDis(const imu_gnss_localizer::Position::ConstPtr& msg){

  UsrPos_Est_enu_x = msg->enu_x;
  UsrPos_Est_enu_y = msg->enu_y;
  UsrPos_Est_enu_z = msg->enu_z;
  UsrPos_index = msg->index;
  //ROS_INFO("UsrPos_Est_enu_x = %f UsrPos_Est_enu_y = %f UsrPos_index = %d" , UsrPos_Est_enu_x,UsrPos_Est_enu_y,UsrPos_index);
}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg){

    ++count;
    IMUTime = IMUperiod * count;
    ROSTime = ros::Time::now().toSec();
    Time = ROSTime; //IMUTime or ROSTime
    //ROS_INFO("Time = %lf" , Time);

    if (BUFNUM < BUFNUM_MAX){
      ++BUFNUM;
    }
    else{
      BUFNUM = BUFNUM_MAX;
    }

    if(T_count == 0 && flag_SF == true && flag_Est == true){
      T_count = 1;
    }
    else if(T_count == 1){
      T_count = 2;
    }

    if (UsrPos_index_Last == UsrPos_index){
      flag_UsrPos_Est = false;
      UsrPos_index = 0;
    }
    else{
      flag_UsrPos_Est = true;
      flag_UsrPos_Start = true;
      UsrPos_index = UsrPos_index;
    }

    if(T_count == 2){
    Est_velE = sin(pHeading) * pVelocity;
    Est_velN = cos(pHeading) * pVelocity;
    p3_msg.VelE = Est_velE;
    p3_msg.VelN = Est_velN;
    p3_msg.VelU = Est_velU;
    p3_msg.index = count;
    }
    pub3.publish(p3_msg);

    if(T_count == 2 && pVelocity > 0 && count > 1){
      Trajectory_x = Trajectory_x_Last + Est_velE * ( Time - Time_Last );
      Trajectory_y = Trajectory_y_Last + Est_velN * ( Time - Time_Last );
      Trajectory_z = Trajectory_z_Last + Est_velU * ( Time - Time_Last );
    }
    else{
      Trajectory_x = Trajectory_x_Last;
      Trajectory_y = Trajectory_y_Last;
      Trajectory_z = Trajectory_z_Last;
    }

    p1_msg.position.x = Trajectory_x;
    p1_msg.position.y = Trajectory_y;
    p1_msg.position.z = Trajectory_z;
    p1_msg.orientation.z = pHeading;

    pUsrPos_Est_enu_x.push_back(UsrPos_Est_enu_x);
    pUsrPos_Est_enu_y.push_back(UsrPos_Est_enu_y);
    pUsrPos_Est_enu_z.push_back(UsrPos_Est_enu_z);
    pTrajectory_x.push_back(Trajectory_x);
    pTrajectory_y.push_back(Trajectory_y);
    pTrajectory_z.push_back(Trajectory_z);

    Est_index = BUFNUM - count - UsrPos_index;

    if(flag_UsrPos_Start == true){

      if(flag_UsrPos_Est == true && Est_index > 0){
        ROS_INFO("diff index = %d" , count-UsrPos_index);
        Est_index = BUFNUM - count - UsrPos_index;
        diff_x = pUsrPos_Est_enu_x[Est_index-1] - UsrPos_Est_enu_x;
        diff_y = pUsrPos_Est_enu_y[Est_index-1] - UsrPos_Est_enu_y;
        diff_z = pUsrPos_Est_enu_z[Est_index-1] - UsrPos_Est_enu_z;
        UsrPos_Est_enu_x = UsrPos_Est_enu_x + diff_x;
        UsrPos_Est_enu_y = UsrPos_Est_enu_y + diff_y;
        UsrPos_Est_enu_z = UsrPos_Est_enu_z + diff_z;
      }
      else if(count > 1){
        UsrPos_Est_enu_x = UsrPos_Est_enu_x_Last + Est_velE * ( Time - Time_Last );
        UsrPos_Est_enu_y = UsrPos_Est_enu_y_Last + Est_velN * ( Time - Time_Last );
        UsrPos_Est_enu_z = UsrPos_Est_enu_z_Last + Est_velU * ( Time - Time_Last );
      }

    }
    p2_msg.position.x = UsrPos_Est_enu_x;
    p2_msg.position.y = UsrPos_Est_enu_y;
    p2_msg.position.z = UsrPos_Est_enu_z;
    p2_msg.orientation.z = pHeading;

    pub1.publish(p1_msg);
    pub2.publish(p2_msg);

    UsrPos_index_Last = UsrPos_index;
    Time_Last = Time;
    Trajectory_x_Last = Trajectory_x;
    Trajectory_y_Last = Trajectory_y;
    Trajectory_z_Last = Trajectory_z;
    UsrPos_Est_enu_x_Last = UsrPos_Est_enu_x;
    UsrPos_Est_enu_y_Last = UsrPos_Est_enu_y;
    UsrPos_Est_enu_z_Last = UsrPos_Est_enu_z;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_Trajectory");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/Heading3rd", 1000, receive_Heading3rd);
  ros::Subscriber sub3 = n.subscribe("/imu_gnss_localizer/PositionDis", 1000, receive_PositionDis);
  ros::Subscriber sub4 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  pub1 = n.advertise<geometry_msgs::Pose>("/imu_gnss_relative_position", 1000);
  pub2 = n.advertise<geometry_msgs::Pose>("/imu_gnss_absolute_position", 1000);
  pub3 = n.advertise<imu_gnss_localizer::UsrVel_enu>("/imu_gnss_localizer/UsrVel_enu", 1000);

  ros::spin();

  return 0;
}
