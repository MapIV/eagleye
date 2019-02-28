/*
 * RCalc_PositionDis_Int.cpp
 * Ego vehicle position realtime estimate program
 * Author Sekino
 * Ver 1.00 2019/2/28
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "imu_gnss_localizer/UsrVel_enu.h"
#include "imu_gnss_localizer/Position.h"
#include <boost/circular_buffer.hpp>

ros::Publisher pub1;

bool flag_UsrPos_Est, flag_UsrPos_Start;
int i;
int count = 0;
int Est_count = 0;
int Est_index = 0;
int UsrPos_index = 0;
int UsrVel_index = 0;
int UsrPos_index_Last = 0;
int BUFNUM = 0;
double IMUTime;
double IMUfrequency = 100; //IMU Hz
double IMUperiod = 1/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
float BUFNUM_MAX = 100;
float UsrVel_enu_E = 0.0;
float UsrVel_enu_N = 0.0;
float UsrVel_enu_U = 0.0;
float UsrPos_Est_enu_x = 0.0;
float UsrPos_Est_enu_y = 0.0;
float UsrPos_Est_enu_z = 0.0;
float UsrPos_EstRaw_enu_x = 0.0;
float UsrPos_EstRaw_enu_y = 0.0;
float UsrPos_EstRaw_enu_z = 0.0;
float UsrPos_Est_enu_x_Last = 0.0;
float UsrPos_Est_enu_y_Last = 0.0;
float UsrPos_Est_enu_z_Last = 0.0;

geometry_msgs::Pose p1_msg;

boost::circular_buffer<float> pUsrPos_Est_enu_x(BUFNUM_MAX);
boost::circular_buffer<float> pUsrPos_Est_enu_y(BUFNUM_MAX);
boost::circular_buffer<float> pUsrPos_Est_enu_z(BUFNUM_MAX);

void receive_PositionDis(const imu_gnss_localizer::Position::ConstPtr& msg){

  UsrPos_EstRaw_enu_x = msg->enu_x;
  UsrPos_EstRaw_enu_y = msg->enu_y;
  UsrPos_EstRaw_enu_z = msg->enu_z;
  UsrPos_index = msg->index;

}

void receive_UsrVel_enu(const imu_gnss_localizer::UsrVel_enu::ConstPtr& msg){

    ++count;
    IMUTime = IMUperiod * count;
    ROSTime = ros::Time::now().toSec();
    Time = ROSTime; //IMUTime or ROSTime
    //ROS_INFO("Time = %lf" , Time);

    UsrVel_enu_E = msg->VelE;
    UsrVel_enu_N = msg->VelN;
    UsrVel_enu_U = msg->VelU;
    UsrVel_index = msg->index;

    if (BUFNUM < BUFNUM_MAX){
      ++BUFNUM;
    }
    else{
      BUFNUM = BUFNUM_MAX;
    }

    if (UsrPos_index_Last == UsrPos_index){
      flag_UsrPos_Est = false;
      UsrPos_index = 0;
      UsrVel_index = 0;
    }
    else{
      flag_UsrPos_Est = true;
      flag_UsrPos_Start = true;
      ++Est_count;
    }

    pUsrPos_Est_enu_x.push_back(UsrPos_Est_enu_x);
    pUsrPos_Est_enu_y.push_back(UsrPos_Est_enu_y);
    pUsrPos_Est_enu_z.push_back(UsrPos_Est_enu_z);

    if(flag_UsrPos_Start == true){

      Est_index = BUFNUM - (UsrVel_index - UsrPos_index);

      if(flag_UsrPos_Est == true && Est_index > 0 && Est_count > 1){
        for(i = Est_index; i < BUFNUM; i++){
        pUsrPos_Est_enu_x[i-1] = pUsrPos_Est_enu_x[i-1] - (pUsrPos_Est_enu_x[Est_index-1] - UsrPos_EstRaw_enu_x);
        pUsrPos_Est_enu_y[i-1] = pUsrPos_Est_enu_y[i-1] - (pUsrPos_Est_enu_y[Est_index-1] - UsrPos_EstRaw_enu_y);
        pUsrPos_Est_enu_z[i-1] = pUsrPos_Est_enu_z[i-1] - (pUsrPos_Est_enu_z[Est_index-1] - UsrPos_EstRaw_enu_z);
        }
        UsrPos_Est_enu_x = pUsrPos_Est_enu_x[BUFNUM-1];
        UsrPos_Est_enu_y = pUsrPos_Est_enu_y[BUFNUM-1];
        UsrPos_Est_enu_z = pUsrPos_Est_enu_z[BUFNUM-1];
        ROS_INFO("Est_index = %d" , Est_index);
      }
      else if(Est_count == 1){
        UsrPos_Est_enu_x = UsrPos_EstRaw_enu_x;
        UsrPos_Est_enu_y = UsrPos_EstRaw_enu_y;
        UsrPos_Est_enu_z = UsrPos_EstRaw_enu_z;
      }
      else if(count > 1){
        UsrPos_Est_enu_x = UsrPos_Est_enu_x_Last + UsrVel_enu_E * ( Time - Time_Last );
        UsrPos_Est_enu_y = UsrPos_Est_enu_y_Last + UsrVel_enu_N * ( Time - Time_Last );
        UsrPos_Est_enu_z = UsrPos_Est_enu_z_Last + UsrVel_enu_U * ( Time - Time_Last );
      }

    }

    if(Est_count > 1){
    p1_msg.position.x = UsrPos_Est_enu_x;
    p1_msg.position.y = UsrPos_Est_enu_y;
    p1_msg.position.z = UsrPos_Est_enu_z;
    }

    pub1.publish(p1_msg);

    Time_Last = Time;
    UsrPos_Est_enu_x_Last = UsrPos_Est_enu_x;
    UsrPos_Est_enu_y_Last = UsrPos_Est_enu_y;
    UsrPos_Est_enu_z_Last = UsrPos_Est_enu_z;
    UsrPos_index_Last = UsrPos_index;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "PositionDis_Int");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/UsrVel_enu", 1000, receive_UsrVel_enu);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/PositionDis", 1000, receive_PositionDis);
  pub1 = n.advertise<geometry_msgs::Pose>("/imu_gnss_pose", 1000);

  ros::spin();

  return 0;
}
