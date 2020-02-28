
#include "ros/ros.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Position.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "rtklib_msgs/RtklibNav.h"
#include "xyz2enu.hpp"

static double estimated_number = 0;
static double estimated_number_max = 5*5; //GNSS output cycle　* Smoothing time
static double estimated_velocity_threshold = 10/3.6;
static double estimated_threshold = 1/10;
static bool specify_base_pos;
static double ecef_base_pos_x;
static double ecef_base_pos_y;
static double ecef_base_pos_z;
static double last_pos[3];
static std::size_t index_length;
static std::size_t time_buffer_length;
static std::size_t velocity_index_length;

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::Position enu_absolute_pos,gnss_smooth_pos_enu;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;

static std::vector<double> time_buffer;
static std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
static std::vector<double> correction_velocity_buffer;

static ros::Publisher pub;



void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  enu_absolute_pos.header = msg->header;
  enu_absolute_pos.enu_pos = msg->enu_pos;
  enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
  enu_absolute_pos.status = msg->status;
}

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;

  if(specify_base_pos == true)
  {
    enu_absolute_pos.ecef_base_pos.x = ecef_base_pos_x;
    enu_absolute_pos.ecef_base_pos.y = ecef_base_pos_y;
    enu_absolute_pos.ecef_base_pos.z = ecef_base_pos_z;
  }
  else if(enu_absolute_pos.ecef_base_pos.x == 0 && enu_absolute_pos.ecef_base_pos.y == 0 && enu_absolute_pos.ecef_base_pos.z == 0)
  {
    enu_absolute_pos.ecef_base_pos.x = msg->ecef_pos.x;
    enu_absolute_pos.ecef_base_pos.y = msg->ecef_pos.y;
    enu_absolute_pos.ecef_base_pos.z = msg->ecef_pos.z;
  }

  double ecef_pos[3];
  double ecef_base_pos[3];
  double enu_pos[3];

  ecef_pos[0] = msg->ecef_pos.x;
  ecef_pos[1] = msg->ecef_pos.y;
  ecef_pos[2] = msg->ecef_pos.z;
  ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
  ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
  ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;

  xyz2enu(ecef_pos, ecef_base_pos, enu_pos);

  ROS_INFO("e = %lf , n = %lf , u = %lf",enu_pos[0],enu_pos[1],enu_pos[2]);

  time_buffer.push_back(msg->header.stamp.toSec());
  enu_pos_x_buffer.push_back(enu_pos[0]);
  enu_pos_y_buffer.push_back(enu_pos[1]);
  enu_pos_z_buffer.push_back(enu_pos[2]);
  correction_velocity_buffer.push_back(velocity_scale_factor.correction_velocity.linear.x);

  time_buffer_length = std::distance(time_buffer.begin(), time_buffer.end());

  if (time_buffer_length > estimated_number_max)
  {
    ROS_INFO("diff_time = %lu",time_buffer_length);
    time_buffer.erase(time_buffer.begin());
    enu_pos_x_buffer.erase(enu_pos_x_buffer.begin());
    enu_pos_y_buffer.erase(enu_pos_y_buffer.begin());
    enu_pos_z_buffer.erase(enu_pos_z_buffer.begin());
    correction_velocity_buffer.erase(correction_velocity_buffer.begin());
  }

  if (estimated_number < estimated_number_max)
  {
    ++estimated_number;
    gnss_smooth_pos_enu.status.enabled_status = false;
  }
  else
  {
    estimated_number = estimated_number_max;
    gnss_smooth_pos_enu.status.enabled_status = true;
  }

  std::vector<int> velocity_index;
  std::vector<int> index;
  int i;
  double gnss_smooth_pos[3] = {0};
  double sum_gnss_pos[3] = {0};


  if (estimated_number == estimated_number_max)
  {
    for (i = 0; i < estimated_number; i++)
    {
      index.push_back(i);
      if (correction_velocity_buffer[i] > estimated_velocity_threshold)
      {
        velocity_index.push_back(i);
      }
    }

    index_length = std::distance(index.begin(), index.end());
    velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

    ROS_INFO("index_length = %lu , velocity_index_length = %lu ",index_length,velocity_index_length);


    for (i = 0; i < velocity_index_length; i++)
    {
      sum_gnss_pos[0] = sum_gnss_pos[0] + enu_pos_x_buffer[velocity_index[i]];
      sum_gnss_pos[1] = sum_gnss_pos[1] + enu_pos_y_buffer[velocity_index[i]];
      sum_gnss_pos[2] = sum_gnss_pos[2] + enu_pos_z_buffer[velocity_index[i]];
    }
    ROS_INFO("s_e = %lf , s_n = %lf , s_u = %lf",sum_gnss_pos[0],sum_gnss_pos[1],sum_gnss_pos[2]);


    if (velocity_index_length > index_length * estimated_threshold)
    {
      gnss_smooth_pos[0] = sum_gnss_pos[0]/velocity_index_length;
      gnss_smooth_pos[1] = sum_gnss_pos[1]/velocity_index_length;
      gnss_smooth_pos[2] = sum_gnss_pos[2]/velocity_index_length;
      gnss_smooth_pos_enu.status.estimate_status = true;
    }
    else
    {
      gnss_smooth_pos[0] = last_pos[0];
      gnss_smooth_pos[1] = last_pos[1];
      gnss_smooth_pos[2] = last_pos[2];
      gnss_smooth_pos_enu.status.estimate_status = false;
    }

    last_pos[0] = gnss_smooth_pos[0];
    last_pos[1] = gnss_smooth_pos[1];
    last_pos[2] = gnss_smooth_pos[2];
  }

  gnss_smooth_pos_enu.enu_pos.x = gnss_smooth_pos[0];
  gnss_smooth_pos_enu.enu_pos.y = gnss_smooth_pos[1];
  gnss_smooth_pos_enu.enu_pos.z = gnss_smooth_pos[2];
  gnss_smooth_pos_enu.ecef_base_pos = enu_absolute_pos.ecef_base_pos;
  gnss_smooth_pos_enu.header = msg->header;
  pub.publish(gnss_smooth_pos_enu);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smoothing");
  ros::NodeHandle n;

  n.getParam("/eagleye/position/specify_base_pos",specify_base_pos);
  n.getParam("/eagleye/position/ecef_base_pos_x",ecef_base_pos_x);
  n.getParam("/eagleye/position/ecef_base_pos_y",ecef_base_pos_y);
  n.getParam("/eagleye/position/ecef_base_pos_z",ecef_base_pos_z);

  std::cout<< "specify_base_pos "<<specify_base_pos<<std::endl;
  std::cout<< "ecef_base_pos_x "<<ecef_base_pos_x<<std::endl;
  std::cout<< "ecef_base_pos_y "<<ecef_base_pos_y<<std::endl;
  std::cout<< "ecef_base_pos_z "<<ecef_base_pos_z<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback);
  ros::Subscriber sub3 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback);

  pub = n.advertise<eagleye_msgs::Position>("/eagleye/gnss_smooth_pos_enu", 1000);

  ros::spin();

  return 0;
}
