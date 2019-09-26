/*
 * heading.cpp
 * heading estimate program
 * Author Sekino
 * Ver 3.00 2019/5/10 Supports extrapolation processing
 * Ver 2.01 2019/4/17 Critical bug fixes
 * Ver 2.00 2019/4/11 Integrate 1st, 2nd, 3rd
 * Ver 1.00 2019/2/6
 */

#include "ros/ros.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/YawrateOffset.h"
#include "rtklib_msgs/RtklibNav.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include "xyz2enu_vel.hpp"
#include <math.h>
#include <numeric>

//default value
bool reverse_imu = false;
double estimated_number_min = 500;
double estimated_number_max = 1500;
double estimated_gnss_coefficient = 1.0 / 10;
double estimated_heading_coefficient = 1.0 / 20;
double outlier_threshold = 3.0 / 180 * M_PI;
double estimated_velocity_threshold = 10 / 3.6;
double stop_judgment_velocity_threshold = 0.01;
double estimated_yawrate_threshold = 5.0 / 180 * M_PI;

bool gnss_status;
int i, count;
int tow_last;
int estimated_number;
int index_max;

double avg = 0.0;
double doppler_heading_angle = 0.0;
double yawrate = 0.0;
double tmp_heading_angle = 0.0;

std::size_t index_length;
std::size_t time_buffer_length;
std::size_t inversion_up_index_length;
std::size_t inversion_down_index_length;

std::vector<double>::iterator max;

std::vector<double> time_buffer;
std::vector<double> heading_angle_buffer;
std::vector<double> yawrate_buffer;
std::vector<double> correction_velocity_buffer;
std::vector<double> yawrate_offset_stop_buffer;
std::vector<double> yawrate_offset_buffer;
std::vector<double> gnss_status_buffer;

rtklib_msgs::RtklibNav rtklib_nav;
eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
eagleye_msgs::YawrateOffset yawrate_offset_stop;
eagleye_msgs::YawrateOffset yawrate_offset;

ros::Publisher pub;
eagleye_msgs::Heading heading;

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;

  double ecef_vel[3];
  double ecef_pos[3];
  double enu_vel[3];

  ecef_vel[0] = msg->ecef_vel.x;
  ecef_vel[1] = msg->ecef_vel.y;
  ecef_vel[2] = msg->ecef_vel.z;
  ecef_pos[0] = msg->ecef_pos.x;
  ecef_pos[1] = msg->ecef_pos.y;
  ecef_pos[2] = msg->ecef_pos.z;

  xyz2enu_vel(ecef_vel, ecef_pos, enu_vel);
  doppler_heading_angle = atan2(enu_vel[0], enu_vel[1]);
}

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void yawrate_offset_stop_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset_stop.header = msg->header;
  yawrate_offset_stop.yawrate_offset = msg->yawrate_offset;
  yawrate_offset_stop.status = msg->status;
}

void yawrate_offset_callback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
{
  yawrate_offset.header = msg->header;
  yawrate_offset.yawrate_offset = msg->yawrate_offset;
  yawrate_offset.status = msg->status;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ++count;

  if (estimated_number < estimated_number_max)
  {
    ++estimated_number;
  }
  else
  {
    estimated_number = estimated_number_max;
  }

  if (reverse_imu == false)
  {
    yawrate = msg->angular_velocity.z;
  }
  else if (reverse_imu == true)
  {
    yawrate = -1 * msg->angular_velocity.z;
  }

  if (tow_last == rtklib_nav.tow)
  {
    gnss_status = false;
    doppler_heading_angle = 0;
    tow_last = rtklib_nav.tow;
  }
  else
  {
    gnss_status = true;
    doppler_heading_angle = doppler_heading_angle;
    tow_last = rtklib_nav.tow;
  }

  // data buffer generate
  time_buffer.push_back(msg->header.stamp.toSec());
  heading_angle_buffer.push_back(doppler_heading_angle);
  yawrate_buffer.push_back(yawrate);
  correction_velocity_buffer.push_back(velocity_scale_factor.correction_velocity.twist.linear.x);
  yawrate_offset_stop_buffer.push_back(yawrate_offset_stop.yawrate_offset);
  yawrate_offset_buffer.push_back(yawrate_offset.yawrate_offset);
  gnss_status_buffer.push_back(gnss_status);

  time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());

  if (time_buffer_length > estimated_number_max)
  {
    time_buffer.erase(time_buffer.begin());
    heading_angle_buffer.erase(heading_angle_buffer.begin());
    yawrate_buffer.erase(yawrate_buffer.begin());
    correction_velocity_buffer.erase(correction_velocity_buffer.begin());
    yawrate_offset_stop_buffer.erase(yawrate_offset_stop_buffer.begin());
    yawrate_offset_buffer.erase(yawrate_offset_buffer.begin());
    gnss_status_buffer.erase(gnss_status_buffer.begin());
  }

  std::vector<int> gnss_index;
  std::vector<int> velocity_index;
  std::vector<int> index;

  if (estimated_number > estimated_number_min && gnss_status_buffer[estimated_number-1] == true &&
      correction_velocity_buffer[estimated_number-1] > estimated_velocity_threshold && fabsf(yawrate_buffer[estimated_number-1]) < estimated_yawrate_threshold)
  {
    heading.status.enabled_status = true;
  }
  else
  {
    heading.status.enabled_status = false;
  }

  if (heading.status.enabled_status == true)
  {
    for (i = 0; i < estimated_number; i++)
    {
      if (gnss_status_buffer[i] == true)
      {
        gnss_index.push_back(i);
      }
      if (correction_velocity_buffer[i] > estimated_velocity_threshold)
      {
        velocity_index.push_back(i);
      }
    }

    set_intersection(gnss_index.begin(), gnss_index.end(), velocity_index.begin(), velocity_index.end(),
                     inserter(index, index.end()));

    index_length = std::distance(index.begin(), index.end());

    if (index_length > estimated_number * estimated_gnss_coefficient)
    {
      std::vector<double> provisional_heading_angle_buffer(estimated_number, 0);

      for (i = 0; i < estimated_number; i++)
      {
        if (i > 0)
        {
          if (correction_velocity_buffer[estimated_number-1] > stop_judgment_velocity_threshold)
          {
            provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] + (yawrate_buffer[i] + yawrate_offset_buffer[i]) * (time_buffer[i] - time_buffer[i-1]);
          }
          else
          {
            provisional_heading_angle_buffer[i] = provisional_heading_angle_buffer[i-1] + (yawrate_buffer[i] + yawrate_offset_stop_buffer[i]) * (time_buffer[i] - time_buffer[i-1]);
          }
        }
      }

      std::vector<double> base_heading_angle_buffer;
      std::vector<double> base_heading_angle_buffer2;
      std::vector<double> diff_buffer;
      std::vector<double> inversion_up_index;
      std::vector<double> inversion_down_index;

      // angle reversal processing (-3.14~3.14)
      for (i = 0; i < estimated_number; i++)
      {
        base_heading_angle_buffer.push_back(heading_angle_buffer[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
      }

      for (i = 0; i < index_length; i++)
      {
        diff_buffer.push_back(base_heading_angle_buffer[index[i]] - heading_angle_buffer[index[i]]);
      }

      for (i = 0; i < index_length; i++)
      {
        if (diff_buffer[i] > M_PI / 2.0)
        {
          inversion_up_index.push_back(index[i]);
        }
        if (diff_buffer[i] < -M_PI / 2.0)
        {
          inversion_down_index.push_back(index[i]);
        }
      }

      inversion_up_index_length = std::distance(inversion_up_index.begin(), inversion_up_index.end());
      inversion_down_index_length = std::distance(inversion_down_index.begin(), inversion_down_index.end());

      if (inversion_up_index_length != 0)
      {
        for (i = 0; i < inversion_up_index_length; i++)
        {
          heading_angle_buffer[inversion_up_index[i]] = heading_angle_buffer[inversion_up_index[i]] + 2.0 * M_PI;
        }
      }

      if (inversion_down_index_length != 0)
      {
        for (i = 0; i < inversion_down_index_length; i++)
        {
          heading_angle_buffer[inversion_down_index[i]] = heading_angle_buffer[inversion_down_index[i]] - 2.0 * M_PI;
        }
      }

      while (1)
      {
        index_length = std::distance(index.begin(), index.end());

        base_heading_angle_buffer.clear();
        for (i = 0; i < estimated_number; i++)
        {
          base_heading_angle_buffer.push_back(heading_angle_buffer[index[index_length-1]] - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
        }

        diff_buffer.clear();
        for (i = 0; i < index_length; i++)
        {
          diff_buffer.push_back(base_heading_angle_buffer[index[i]] - heading_angle_buffer[index[i]]);
        }

        avg = std::accumulate(diff_buffer.begin(), diff_buffer.end(), 0.0) / index_length;
        tmp_heading_angle = heading_angle_buffer[index[index_length-1]] - avg;

        base_heading_angle_buffer2.clear();
        for (i = 0; i < estimated_number; i++)
        {
          base_heading_angle_buffer2.push_back(tmp_heading_angle - provisional_heading_angle_buffer[index[index_length-1]] + provisional_heading_angle_buffer[i]);
        }

        diff_buffer.clear();
        for (i = 0; i < index_length; i++)
        {
          diff_buffer.push_back(fabsf(base_heading_angle_buffer2[index[i]] - heading_angle_buffer[index[i]]));
        }

        max = std::max_element(diff_buffer.begin(), diff_buffer.end());
        index_max = std::distance(diff_buffer.begin(), max);

        if (diff_buffer[index_max] > outlier_threshold)
        {
          index.erase(index.begin() + index_max);
        }
        else
        {
          break;
        }

        index_length = std::distance(index.begin(), index.end());

        if (index_length < estimated_number * estimated_heading_coefficient)
        {
          break;
        }
      }

      if (index_length == 0 || index_length > estimated_number * estimated_heading_coefficient)
      {
        if (index[index_length-1] == estimated_number-1)
        {
          heading.heading_angle = tmp_heading_angle;
        }
        else
        {
          heading.heading_angle = tmp_heading_angle + (provisional_heading_angle_buffer[estimated_number-1] - provisional_heading_angle_buffer[index[index_length-1]]);
        }

        if (heading.heading_angle > M_PI)
        {
          heading.heading_angle = heading.heading_angle - 2.0 * M_PI;
        }
        else if (heading.heading_angle < -M_PI)
        {
          heading.heading_angle = heading.heading_angle + 2.0 * M_PI;
        }
        heading.header = msg->header;
        heading.status.estimate_status = true;
        pub.publish(heading);
      }
    }
  }
  heading.status.estimate_status = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "heading");
  ros::NodeHandle n("~");
  n.getParam("/eagleye/reverse_imu", reverse_imu);
  n.getParam("/eagleye/heading/estimated_number_min",estimated_number_min);
  n.getParam("/eagleye/heading/estimated_number_max",estimated_number_max);
  n.getParam("/eagleye/heading/estimated_gnss_coefficient",estimated_gnss_coefficient);
  n.getParam("/eagleye/heading/estimated_heading_coefficient",estimated_heading_coefficient);
  n.getParam("/eagleye/heading/outlier_threshold",outlier_threshold);
  n.getParam("/eagleye/heading/estimated_velocity_threshold",estimated_velocity_threshold);
  n.getParam("/eagleye/heading/stop_judgment_velocity_threshold",stop_judgment_velocity_threshold);
  n.getParam("/eagleye/heading/estimated_yawrate_threshold",estimated_yawrate_threshold);

  std::cout<< "reverse_imu "<<reverse_imu<<std::endl;
  std::cout<< "estimated_number_min "<<estimated_number_min<<std::endl;
  std::cout<< "estimated_number_max "<<estimated_number_max<<std::endl;
  std::cout<< "estimated_gnss_coefficient "<<estimated_gnss_coefficient<<std::endl;
  std::cout<< "estimated_heading_coefficient "<<estimated_heading_coefficient<<std::endl;
  std::cout<< "outlier_threshold "<<outlier_threshold<<std::endl;
  std::cout<< "estimated_velocity_threshold "<<estimated_velocity_threshold<<std::endl;
  std::cout<< "stop_judgment_velocity_threshold "<<stop_judgment_velocity_threshold<<std::endl;
  std::cout<< "estimated_yawrate_threshold "<<estimated_yawrate_threshold<<std::endl;

  std::string publish_topic_name = "/publish_topic_name/invalid";
  std::string subscribe_topic_name = "/subscribe_topic_name/invalid";

  if (argc == 2)
  {
    if (strcmp(argv[1], "1st") == 0)
    {
      publish_topic_name = "/eagleye/heading_1st";
      subscribe_topic_name = "/eagleye/yawrate_offset_stop";
    }
    else if (strcmp(argv[1], "2nd") == 0)
    {
      publish_topic_name = "/eagleye/heading_2nd";
      subscribe_topic_name = "/eagleye/yawrate_offset_1st";
    }
    else if (strcmp(argv[1], "3rd") == 0)
    {
      publish_topic_name = "/eagleye/heading_3rd";
      subscribe_topic_name = "/eagleye/yawrate_offset_2nd";
    }
    else
    {
      ROS_ERROR("Invalid argument");
      ros::shutdown();
    }
  }
  else
  {
    ROS_ERROR("No arguments");
    ros::shutdown();
  }

  ros::Subscriber sub1 = n.subscribe("/imu/data_raw", 1000, imu_callback);
  ros::Subscriber sub2 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub4 = n.subscribe("/eagleye/yawrate_offset_stop", 1000, yawrate_offset_stop_callback);
  ros::Subscriber sub5 = n.subscribe(subscribe_topic_name, 1000, yawrate_offset_callback);

  pub = n.advertise<eagleye_msgs::Heading>(publish_topic_name, 1000);

  ros::spin();

  return 0;
}
