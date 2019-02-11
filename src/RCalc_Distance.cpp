/*
 * RCalc_Distance.cpp
 * Distance estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/data.h"

ros::Publisher pub;

bool flag_Est, flag_SF;
int count = 0;
int T_count = 0;
int GPSTime_Last, GPSTime;
int ESTNUM_Heading = 0;
double IMUTime;
double IMUfrequency = 100; //IMU Hz
double IMUperiod = 1/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
float pVelocity = 0.0;
float Distance = 0.0;
float Distance_Last = 0.0;

std_msgs::Float64 p_msg;

void receive_VelocitySF(const imu_gnss_localizer::data::ConstPtr& msg){

    pVelocity = msg->EstimateValue;

}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg){

    ++count;
    IMUTime = IMUperiod * count;
    ROSTime = ros::Time::now().toSec();
    Time = ROSTime; //IMUTime or ROSTime
    //ROS_INFO("Time = %lf" , Time);

    Distance = Distance_Last + pVelocity * ( Time - Time_Last );

    p_msg.data = Distance;
    pub.publish(p_msg);

    Time_Last = Time;
    Distance_Last = Distance;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_Distance");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  pub = n.advertise<std_msgs::Float64>("/imu_gnss_localizer/Distance", 1000);

  ros::spin();

  return 0;
}
