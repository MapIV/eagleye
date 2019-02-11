/*
 * RCalc_Trajectory.cpp
 * Trajectory estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/data.h"

ros::Publisher pub;

bool flag_Est, flag_SF;
int count = 0;
int T_count = 0;
double IMUTime;
double IMUfrequency = 100; //IMU Hz
double IMUperiod = 1/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
float Est_velE = 0.0;
float Est_velN = 0.0;
float Est_velU = 0.0;
float Trajectory_x = 0.0;
float Trajectory_y = 0.0;
float Trajectory_z = 0.0;
float Trajectory_x_Last = 0.0;
float Trajectory_y_Last = 0.0;
float Trajectory_z_Last = 0.0;
float pVelocity = 0.0;
float pHeading = 0.0;

geometry_msgs::Pose p_msg;

void receive_VelocitySF(const imu_gnss_localizer::data::ConstPtr& msg){

  pVelocity = msg->EstimateValue;
  flag_SF = msg->Flag;

}

void receive_Heading3rd(const imu_gnss_localizer::data::ConstPtr& msg){

  pHeading = msg->EstimateValue;
  flag_Est = msg->Flag;

}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg){

    ++count;
    IMUTime = IMUperiod * count;
    ROSTime = ros::Time::now().toSec();
    Time = ROSTime; //IMUTime or ROSTime
    //ROS_INFO("Time = %lf" , Time);

    if(T_count == 0 && flag_SF == true && flag_Est == true){
      T_count = 1;
    }
    else if(T_count == 1){
      T_count = 2;
    }

    Est_velE = sin(pHeading) * pVelocity;
    Est_velN = cos(pHeading) * pVelocity;

    if(T_count == 2 && pVelocity > 0){
      Trajectory_x = Trajectory_x_Last + Est_velE * ( Time - Time_Last );
      Trajectory_y = Trajectory_y_Last + Est_velN * ( Time - Time_Last );
    }
    else{
      Trajectory_x = Trajectory_x_Last;
      Trajectory_y = Trajectory_y_Last;
    }

    p_msg.position.x = Trajectory_x;
    p_msg.position.y = Trajectory_y;
    p_msg.orientation.z = pHeading;
    pub.publish(p_msg);

    Time_Last = Time;
    Trajectory_x_Last = Trajectory_x;
    Trajectory_y_Last = Trajectory_y;
    Trajectory_z_Last = Trajectory_z;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_Trajectory");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/Heading3rd", 1000, receive_Heading3rd);
  ros::Subscriber sub3 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  pub = n.advertise<geometry_msgs::Pose>("/imu_gnss_localizer/Trajectory", 1000);

  ros::spin();

  return 0;
}
