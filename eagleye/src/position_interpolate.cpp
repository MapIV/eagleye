/*
 * position_interpolate.cpp
 * Vehicle position realtime estimate program
 * Author Sekino
 * Ver 1.01 2019/5/9 Change from index to sync with timestamp
 * Ver 1.00 2019/2/28
 */

#include "ros/ros.h"
#include "eagleye_msgs/Position.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "enu2llh.hpp"
#include <boost/circular_buffer.hpp>

//default value
double number_buffer_max = 100;

bool position_estimate_status, position_estimate_start_status;
int i, count, position_estimate_status_count;
int estimate_index = 0;
int number_buffer = 0;

double position_stamp_last = 0;
double time_last = 0.0;
double provisional_enu_pos_x = 0.0;
double provisional_enu_pos_y = 0.0;
double provisional_enu_pos_z = 0.0;
double diff_estimate_enu_pos_x = 0.0;
double diff_estimate_enu_pos_y = 0.0;
double diff_estimate_enu_pos_z = 0.0;
double estimate_enu_pos_x_last = 0.0;
double estimate_enu_pos_y_last = 0.0;
double estimate_enu_pos_z_last = 0.0;

boost::circular_buffer<double> provisional_enu_pos_x_buffer(number_buffer_max);
boost::circular_buffer<double> provisional_enu_pos_y_buffer(number_buffer_max);
boost::circular_buffer<double> provisional_enu_pos_z_buffer(number_buffer_max);
boost::circular_buffer<double> imu_stamp_buffer(number_buffer_max);

eagleye_msgs::Position enu_absolute_pos;

eagleye_msgs::Position enu_absolute_pos_interpolate;
sensor_msgs::NavSatFix eagleye_fix;
ros::Publisher pub1;
ros::Publisher pub2;

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  enu_absolute_pos_interpolate.header = msg->header;
  enu_absolute_pos_interpolate.ecef_base_pos = enu_absolute_pos.ecef_base_pos;
  eagleye_fix.header = msg->header;

  ++count;

  if (number_buffer < number_buffer_max)
  {
    ++number_buffer;
  }
  else
  {
    number_buffer = number_buffer_max;
  }

  if (position_stamp_last == enu_absolute_pos.header.stamp.toSec())
  {
    position_estimate_status = false;
  }
  else
  {
    position_estimate_status = true;
    position_estimate_start_status = true;
    ++position_estimate_status_count;
  }

  provisional_enu_pos_x = estimate_enu_pos_x_last + msg->vector.x * (msg->header.stamp.toSec() - time_last);
  provisional_enu_pos_y = estimate_enu_pos_y_last + msg->vector.y * (msg->header.stamp.toSec() - time_last);
  provisional_enu_pos_z = estimate_enu_pos_z_last + msg->vector.z * (msg->header.stamp.toSec() - time_last);

  provisional_enu_pos_x_buffer.push_back(provisional_enu_pos_x);
  provisional_enu_pos_y_buffer.push_back(provisional_enu_pos_y);
  provisional_enu_pos_z_buffer.push_back(provisional_enu_pos_z);
  imu_stamp_buffer.push_back(msg->header.stamp.toSec());

  if (position_estimate_start_status == true)
  {
    if (position_estimate_status == true)
    {
      for (estimate_index = number_buffer; estimate_index > 0; estimate_index--)
      {
        if (imu_stamp_buffer[estimate_index-1] == enu_absolute_pos.header.stamp.toSec())
        {
          break;
        }
      }
    }

    if (position_estimate_status == true && estimate_index > 0 && number_buffer >= estimate_index && position_estimate_status_count > 1)
    {
      diff_estimate_enu_pos_x = (provisional_enu_pos_x_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.x);
      diff_estimate_enu_pos_y = (provisional_enu_pos_y_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.y);
      diff_estimate_enu_pos_z = (provisional_enu_pos_z_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.z);
      for (i = estimate_index; i <= number_buffer; i++)
      {
        provisional_enu_pos_x_buffer[i-1] = provisional_enu_pos_x_buffer[i-1] - diff_estimate_enu_pos_x;
        provisional_enu_pos_y_buffer[i-1] = provisional_enu_pos_y_buffer[i-1] - diff_estimate_enu_pos_y;
        provisional_enu_pos_z_buffer[i-1] = provisional_enu_pos_z_buffer[i-1] - diff_estimate_enu_pos_z;
      }
      provisional_enu_pos_x = provisional_enu_pos_x_buffer[number_buffer-1];
      provisional_enu_pos_y = provisional_enu_pos_y_buffer[number_buffer-1];
      provisional_enu_pos_z = provisional_enu_pos_z_buffer[number_buffer-1];

      enu_absolute_pos_interpolate.status.enabled_status = true;
      enu_absolute_pos_interpolate.status.estimate_status = true;
    }

    else if (position_estimate_status_count == 1)
    {
      provisional_enu_pos_x = enu_absolute_pos_interpolate.enu_pos.x;
      provisional_enu_pos_y = enu_absolute_pos_interpolate.enu_pos.y;
      provisional_enu_pos_z = enu_absolute_pos_interpolate.enu_pos.z;

      enu_absolute_pos_interpolate.status.enabled_status = true;
      enu_absolute_pos_interpolate.status.estimate_status = true;
    }
    else if (count > 1)
    {
      enu_absolute_pos_interpolate.status.estimate_status = false;
    }
  }

  if (position_estimate_status_count > 1)
  {
    double enu_pos[3];
    double ecef_base_pos[3];

    enu_pos[0] = enu_absolute_pos_interpolate.enu_pos.x;
    enu_pos[1] = enu_absolute_pos_interpolate.enu_pos.y;
    enu_pos[2] = enu_absolute_pos_interpolate.enu_pos.z;
    ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;

    enu2llh(enu_pos, ecef_base_pos, eagleye_fix.longitude, eagleye_fix.latitude, eagleye_fix.altitude);

    pub2.publish(eagleye_fix);

    enu_absolute_pos_interpolate.enu_pos.x = provisional_enu_pos_x;
    enu_absolute_pos_interpolate.enu_pos.y = provisional_enu_pos_y;
    enu_absolute_pos_interpolate.enu_pos.z = provisional_enu_pos_z;
  }
  pub1.publish(enu_absolute_pos_interpolate);

  estimate_enu_pos_x_last = provisional_enu_pos_x;
  estimate_enu_pos_y_last = provisional_enu_pos_y;
  estimate_enu_pos_z_last = provisional_enu_pos_z;
  position_stamp_last = enu_absolute_pos.header.stamp.toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_interpolate");
  ros::NodeHandle n;

  n.getParam("/eagleye/position_interpolate/number_buffer_max", number_buffer_max);
  std::cout<< "number_buffer_max "<<number_buffer_max<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/enu_vel", 1000, enu_vel_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback);
  pub1 = n.advertise<eagleye_msgs::Position>("/eagleye/enu_absolute_pos_interpolate", 1000);
  pub2 = n.advertise<sensor_msgs::NavSatFix>("/eagleye/fix", 1000);

  ros::spin();

  return 0;
}
