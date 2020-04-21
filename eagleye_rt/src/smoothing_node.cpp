
#include "ros/ros.h"
#include "eagleye_msgs/VelocityScaleFactor.h"
#include "eagleye_msgs/Position.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "rtklib_msgs/RtklibNav.h"
#include "coordinate.hpp"
#include "navigation.hpp"


static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::Position enu_absolute_pos,gnss_smooth_pos_enu;
static eagleye_msgs::VelocityScaleFactor velocity_scale_factor;
static ros::Publisher pub;

struct PositionParam smoothing_param;
struct SmoothingStatus smoothing_status;

void velocity_scale_factor_callback(const eagleye_msgs::VelocityScaleFactor::ConstPtr& msg)
{
  velocity_scale_factor.header = msg->header;
  velocity_scale_factor.scale_factor = msg->scale_factor;
  velocity_scale_factor.correction_velocity = msg->correction_velocity;
  velocity_scale_factor.status = msg->status;
}

// void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
// {
//   enu_absolute_pos.header = msg->header;
//   enu_absolute_pos.enu_pos = msg->enu_pos;
//   enu_absolute_pos.ecef_base_pos = msg->ecef_base_pos;
//   enu_absolute_pos.status = msg->status;
// }

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav.header = msg->header;
  rtklib_nav.tow = msg->tow;
  rtklib_nav.ecef_pos = msg->ecef_pos;
  rtklib_nav.ecef_vel = msg->ecef_vel;
  rtklib_nav.status = msg->status;
  calc_smoothing(rtklib_nav,velocity_scale_factor,smoothing_param,&smoothing_status,&gnss_smooth_pos_enu);
  gnss_smooth_pos_enu.header = msg->header;
  pub.publish(gnss_smooth_pos_enu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smoothing");
  ros::NodeHandle n;

  n.getParam("/eagleye/position/ecef_base_pos_x",smoothing_param.ecef_base_pos_x);
  n.getParam("/eagleye/position/ecef_base_pos_y",smoothing_param.ecef_base_pos_y);
  n.getParam("/eagleye/position/ecef_base_pos_z",smoothing_param.ecef_base_pos_z);

  std::cout<< "ecef_base_pos_x "<<smoothing_param.ecef_base_pos_x<<std::endl;
  std::cout<< "ecef_base_pos_y "<<smoothing_param.ecef_base_pos_y<<std::endl;
  std::cout<< "ecef_base_pos_z "<<smoothing_param.ecef_base_pos_z<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/velocity_scale_factor", 1000, velocity_scale_factor_callback, ros::TransportHints().tcpNoDelay());
  // ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());

  pub = n.advertise<eagleye_msgs::Position>("/eagleye/gnss_smooth_pos_enu", 1000);

  ros::spin();

  return 0;
}
