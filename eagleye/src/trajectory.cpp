/*
 * trajectory.cpp
 * Trajectory estimate program
 * Author Sekino
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "eagleye_msgs/Position.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

//default value
bool reverse_imu = false;
double stop_judgment_velocity_threshold = 0.01;

int count, estimate_status_count;
double time_last = 0.0;

eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
eagleye_msgs::Heading heading_interpolate_3rd;
eagleye_msgs::YawrateOffset yawrate_offset_stop;
eagleye_msgs::YawrateOffset yawrate_offset_2nd;

geometry_msgs::Vector3Stamped enu_vel;
eagleye_msgs::Position enu_relative_pos;
geometry_msgs::TwistStamped eagleye_twist;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  heading_interpolate_3rd.header = msg->header;
  heading_interpolate_3rd.heading_angle = msg->heading_angle;
  heading_interpolate_3rd.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void yawrate_offset_2nd_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_2nd.header = msg->header;
  yawrate_offset_2nd.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_2nd.status = msg->status;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  enu_vel.header = msg->header;
  enu_relative_pos.header = msg->header;
  eagleye_twist.header = msg->header;

  ++count;

  if (reverse_imu == false)
  {
    if (velocity_scale_factor.correction_velocity.twist.linear.x > stop_judgment_velocity_threshold && yawrate_offset_2nd.status.enabled_status == true)
    {
      eagleye_twist.twist.angular.z = -1 * msg->angular_velocity.z + yawrate_offset_2nd.yawrate_offset; //Inverted because the coordinate system is reversed
    }
    else
    {
      eagleye_twist.twist.angular.z = -1 * msg->angular_velocity.z + yawrate_offset_stop.yawrate_offset; //Inverted because the coordinate system is reversed
    }
  }
  else if (velocity_scale_factor.correction_velocity.twist.linear.x > stop_judgment_velocity_threshold && yawrate_offset_2nd.status.enabled_status == true)
  {
    if (velocity_scale_factor.correction_velocity.twist.linear.x > stop_judgment_velocity_threshold)
    {
      eagleye_twist.twist.angular.z = -1 * (-1 * msg->angular_velocity.z) + yawrate_offset_2nd.yawrate_offset; //Inverted because the coordinate system is reversed
    }
    else
    {
      eagleye_twist.twist.angular.z = -1 * (-1 * msg->angular_velocity.z) + yawrate_offset_stop.yawrate_offset; //Inverted because the coordinate system is reversed
    }
  }
  eagleye_twist.twist.linear.x = velocity_scale_factor.correction_velocity.twist.linear.x;
  pub3.publish(eagleye_twist);

  if (estimate_status_count == 0 && velocity_scale_factor.status.enabled_status == true && heading_interpolate_3rd.status.enabled_status == true)
  {
    estimate_status_count = 1;
  }
  else if (estimate_status_count == 1)
  {
    estimate_status_count = 2;
  }

  if (estimate_status_count == 2)
  {
    enu_vel.vector.x = sin(heading_interpolate_3rd.heading_angle) * velocity_scale_factor.correction_velocity.twist.linear.x; //vel_e
    enu_vel.vector.y = cos(heading_interpolate_3rd.heading_angle) * velocity_scale_factor.correction_velocity.twist.linear.x; //vel_n
    enu_vel.vector.z = 0; //vel_u
  }
  pub1.publish(enu_vel);

  if (estimate_status_count == 2 && velocity_scale_factor.correction_velocity.twist.linear.x > 0 && count > 1)
  {
    enu_relative_pos.enu_pos.x = enu_relative_pos.enu_pos.x + enu_vel.vector.x * (msg->header.stamp.toSec() - time_last);
    enu_relative_pos.enu_pos.y = enu_relative_pos.enu_pos.y + enu_vel.vector.y * (msg->header.stamp.toSec() - time_last);
    enu_relative_pos.enu_pos.z = 0;
  }
  pub2.publish(enu_relative_pos);

  time_last = msg->header.stamp.toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory");

  ros::NodeHandle n;

  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/Trajectory/stop_judgment_velocity_threshold",stop_judgment_velocity_threshold);
  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback);
  ros::Subscriber sub4 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  ros::Subscriber sub5 = n.subscribe("/eagleye/yawrate_offset_2nd", 1000, yawrate_offset_2nd_callback);
  pub1 = n.advertise<geometry_msgs::Vector3Stamped>("/eagleye/enu_vel", 1000);
  pub2 = n.advertise<eagleye_msgs::Position>("/eagleye/enu_relative_pos", 1000);
  pub3 = n.advertise<geometry_msgs::TwistStamped>("/eagleye/twist", 1000);

  ros::spin();

  return 0;
}
