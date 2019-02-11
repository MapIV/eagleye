/*
 * RCalc_Trajectory.cpp
 * Trajectory estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/data.h"

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;

bool flag_Est, flag_SF;
int count = 0;
int T_count = 0;
float pi = 3.141592653589793;
double IMUTime;
double IMUfrequency = 100; //IMU Hz
double IMUperiod = 1/IMUfrequency;
double ROSTime = 0.0;
double Time = 0.0;
double Time_Last = 0.0;
float Est_velE = 0.0;
float Est_velN = 0.0;
float Est_velU = 0.0;
float Est_velE_YOS_only = 0.0;
float Est_velN_YOS_only = 0.0;
float Est_velU_YOS_only = 0.0;
float Est_velE_without_logic = 0.0;
float Est_velN_without_logic = 0.0;
float Est_velU_without_logic = 0.0;
float Trajectory_x = 0.0;
float Trajectory_y = 0.0;
float Trajectory_z = 0.0;
float Trajectory_x_YOS_only = 0.0;
float Trajectory_y_YOS_only = 0.0;
float Trajectory_z_YOS_only = 0.0;
float Trajectory_x_without_logic = 0.0;
float Trajectory_y_without_logic = 0.0;
float Trajectory_z_without_logic = 0.0;
float Trajectory_x_Last = 0.0;
float Trajectory_y_Last = 0.0;
float Trajectory_z_Last = 0.0;
float Trajectory_x_Last_YOS_only = 0.0;
float Trajectory_y_Last_YOS_only = 0.0;
float Trajectory_z_Last_YOS_only = 0.0;
float Trajectory_x_Last_without_logic = 0.0;
float Trajectory_y_Last_without_logic = 0.0;
float Trajectory_z_Last_without_logic = 0.0;
float pVelocity = 0.0;
float Velocity = 0.0;
float pHeading = 0.0;
float pHeading_YOS_only = 0.0;
float pHeading_YOS_only_Last = 0.0;
float pHeading_without_logic = 0.0;
float pHeading_without_logic_Last = 0.0;
float Yawrate = 0.0;
float YawrateOffset_Stop = 0.0;
float initpHeading = 0.0;

geometry_msgs::Pose p_msg1;
geometry_msgs::Pose p_msg2;
geometry_msgs::Pose p_msg3;

void receive_Velocity(const geometry_msgs::Twist::ConstPtr& msg){

    Velocity = msg->linear.x ;

}

void receive_YawrateOffsetStop(const imu_gnss_localizer::data::ConstPtr& msg){

  YawrateOffset_Stop = msg->EstimateValue;

}

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

    Yawrate = -1 * msg->angular_velocity.z;

    if(T_count == 0 && flag_SF == true && flag_Est == true){
      T_count = 1;
      pHeading_YOS_only = pHeading;
      pHeading_without_logic = pHeading;
    }
    else if(T_count == 1){
      T_count = 2;
    }

    Est_velE = sin(pHeading) * pVelocity;
    Est_velN = cos(pHeading) * pVelocity;

    if(T_count ==2){
    pHeading_YOS_only = fmod(pHeading_YOS_only_Last  + (Yawrate + YawrateOffset_Stop) * ( Time - Time_Last ) , 2*pi)  ;
    pHeading_without_logic = fmod(pHeading_without_logic_Last  + Yawrate * ( Time - Time_Last ) , 2*pi );
    }

    Est_velE_YOS_only = sin(pHeading_YOS_only) * Velocity;
    Est_velN_YOS_only = cos(pHeading_YOS_only) * Velocity;

    Est_velE_without_logic = sin(pHeading_without_logic) * Velocity;
    Est_velN_without_logic = cos(pHeading_without_logic) * Velocity;

    if(T_count == 2 && pVelocity > 0){
      Trajectory_x = Trajectory_x_Last + Est_velE * ( Time - Time_Last );
      Trajectory_y = Trajectory_y_Last + Est_velN * ( Time - Time_Last );

      Trajectory_x_YOS_only = Trajectory_x_Last_YOS_only + Est_velE_YOS_only * ( Time - Time_Last );
      Trajectory_y_YOS_only = Trajectory_y_Last_YOS_only + Est_velN_YOS_only * ( Time - Time_Last );

      Trajectory_x_without_logic = Trajectory_x_Last_without_logic + Est_velE_without_logic * ( Time - Time_Last );
      Trajectory_y_without_logic = Trajectory_y_Last_without_logic + Est_velN_without_logic * ( Time - Time_Last );
    }
    else{
      Trajectory_x = Trajectory_x_Last;
      Trajectory_y = Trajectory_y_Last;

      Trajectory_x_YOS_only = Trajectory_x_Last_YOS_only;
      Trajectory_y_YOS_only = Trajectory_y_Last_YOS_only;

      Trajectory_x_without_logic = Trajectory_x_Last_without_logic;
      Trajectory_y_without_logic = Trajectory_y_Last_without_logic;
    }

    p_msg1.position.x = Trajectory_x;
    p_msg1.position.y = Trajectory_y;
    p_msg1.orientation.z = pHeading;
    pub1.publish(p_msg1);

    p_msg2.position.x = Trajectory_x_YOS_only;
    p_msg2.position.y = Trajectory_y_YOS_only;
    p_msg2.orientation.z = pHeading_YOS_only;
    pub2.publish(p_msg2);

    p_msg3.position.x = Trajectory_x_without_logic;
    p_msg3.position.y = Trajectory_y_without_logic;
    p_msg3.orientation.z = pHeading_without_logic;
    pub3.publish(p_msg3);

    Time_Last = Time;
    Trajectory_x_Last = Trajectory_x;
    Trajectory_y_Last = Trajectory_y;
    Trajectory_z_Last = Trajectory_z;

    Trajectory_x_Last_YOS_only = Trajectory_x_YOS_only;
    Trajectory_y_Last_YOS_only = Trajectory_y_YOS_only;
    Trajectory_z_Last_YOS_only = Trajectory_z_YOS_only;

    Trajectory_x_Last_without_logic = Trajectory_x_without_logic;
    Trajectory_y_Last_without_logic = Trajectory_y_without_logic;
    Trajectory_z_Last_without_logic = Trajectory_z_without_logic;

    pHeading_YOS_only_Last = pHeading_YOS_only;
    pHeading_without_logic_Last = pHeading_without_logic;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_Trajectory_compare");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/imu_gnss_localizer/VelocitySF", 1000, receive_VelocitySF);
  ros::Subscriber sub2 = n.subscribe("/imu_gnss_localizer/Heading3rd", 1000, receive_Heading3rd);
  ros::Subscriber sub3 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  ros::Subscriber sub4 = n.subscribe("/Vehicle/Velocity", 1000, receive_Velocity);
  ros::Subscriber sub5 = n.subscribe("/imu_gnss_localizer/YawrateOffsetStop", 1000, receive_YawrateOffsetStop);
  pub1 = n.advertise<geometry_msgs::Pose>("/imu_gnss_localizer/Trajectory", 1000);
  pub2 = n.advertise<geometry_msgs::Pose>("/imu_gnss_localizer/Trajectory_YOS_only", 1000);
  pub3 = n.advertise<geometry_msgs::Pose>("/imu_gnss_localizer/Trajectory_without_logic", 1000);

  ros::spin();

  return 0;
}
