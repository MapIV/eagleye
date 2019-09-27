/*
 * distance.cpp
 * Distance estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "eagleye_msgs/Distance.h"
#include "eagleye_msgs/VelocityScaleFactor.h"

int count;
double time_last;

ros::Publisher pub;
eagleye_msgs::Distance distance;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  ++count;
  distance.header = msg->header;

  if (count > 2)
  {
    distance.distance = distance.distance + msg->correction_velocity.linear.x * (msg->header.stamp.toSec() - time_last);
    pub.publish(distance);
    time_last = msg->header.stamp.toSec();
  }
  else
  {
    time_last = msg->header.stamp.toSec();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distance");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  pub = n.advertise<eagleye_msgs::Distance>("/eagleye/distance", 1000);

  ros::spin();

  return 0;
}
