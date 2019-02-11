/*
 * RCalc_YawrateOffsetStop.cpp
 * yawrate offset stop estimate program
 * Author Sekino
 * Ver 1.00 2019/1/24
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "imu_gnss_localizer/data.h"
#include <boost/circular_buffer.hpp>

ros::Publisher pub;

bool flag_YOS, flag_YOSRaw;
int count_vel = 0;
float TH_VEL = 0.01;
float TH_VEL_STOP = 200;
float Velocity = 0.0;
float YawrateOffset_Stop = 0.0;
float YO_Stop_Last = 0.0;

imu_gnss_localizer::data p_msg;
boost::circular_buffer<float> pYawrate(TH_VEL_STOP);

void receive_Velocity(const geometry_msgs::Twist::ConstPtr& msg){

  Velocity = msg->linear.x ;

}

void receive_Imu(const sensor_msgs::Imu::ConstPtr& msg){

    //data buffer generate
  	pYawrate.push_back(-1 * msg->angular_velocity.z);

    //vehicle stop time counter
  if(Velocity < TH_VEL){
    ++count_vel;
  	}
  else{
    count_vel = 0;
  	}

    //yawrate offset calculation (mean)
  if(count_vel > TH_VEL_STOP){
    float tmp = 0.0;
    for(int i = 0; i < TH_VEL_STOP; i++){
    tmp += pYawrate[i];
    }
    YawrateOffset_Stop = -1 * tmp/TH_VEL_STOP;
    flag_YOS = true;
    flag_YOSRaw = true;
  	}
  else{
    YawrateOffset_Stop = YO_Stop_Last;
    flag_YOSRaw = false;
  	}

  //ROS_INFO("YawrateOffset_Stop = %f", YawrateOffset_Stop );

  p_msg.EstimateValue = YawrateOffset_Stop;
  p_msg.Flag = flag_YOS;
  p_msg.EstFlag = flag_YOSRaw;
  pub.publish(p_msg);

  YO_Stop_Last = YawrateOffset_Stop;

}

int main(int argc, char **argv){

  ros::init(argc, argv, "RCalc_YawrateOffsetStop");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/Vehicle/Velocity", 1000, receive_Velocity);
  ros::Subscriber sub2 = n.subscribe("/imu/data_raw", 1000, receive_Imu);
  pub = n.advertise<imu_gnss_localizer::data>("/imu_gnss_localizer/YawrateOffsetStop", 1000);

  ros::spin();

  return 0;
}
