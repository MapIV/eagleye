
#include "ros/ros.h"
#include "gnss_converter/nmea2fix.hpp"
#include "rtklib_msgs/RtklibNav.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <coordinate/coordinate.hpp>
#include <ublox_msgs/NavPVT.h>

static ros::Publisher pub1,pub2, pub3, pub4, pub5;
static nmea_msgs::Sentence sentence;
sensor_msgs::NavSatFix::ConstPtr nav_msg_ptr;

static std::string sub_topic_name,pub_fix_topic_name,pub_gga_topic_name, pub_rmc_topic_name;
static bool output_gga, output_rmc;

void nmea_callback(const nmea_msgs::Sentence::ConstPtr &msg)
{
  nmea_msgs::Gpgga gga;
  nmea_msgs::Gprmc rmc;
  sensor_msgs::NavSatFix fix;

  sentence.header = msg->header;
  sentence.sentence = msg->sentence;
  nmea2fix_converter(sentence, &fix, &gga, &rmc);
  if (fix.header.stamp.toSec() != 0)
  {
    gga.header.frame_id = fix.header.frame_id = "gnss";
    pub1.publish(fix);
    pub2.publish(gga);
  }
  if (rmc.header.stamp.toSec() != 0)
  {
    rmc.header.frame_id = "gnss";
    pub3.publish(rmc);
  }
}

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr &msg) {
  pub4.publish(*msg);;
}

void navsatfix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) { nav_msg_ptr = msg; }

void navpvt_callback(const ublox_msgs::NavPVT::ConstPtr& msg)
{
  rtklib_msgs::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp.sec = msg->sec;
  r.header.stamp.nsec = msg->nano;
 if (nav_msg_ptr != nullptr)
    r.status = *nav_msg_ptr;
  r.tow = msg->iTOW;

  double llh[3];
  llh[0] = msg->lat * 1e-7 * M_PI / 180.0;  // [deg / 1e-7]->[rad]
  llh[1] = msg->lon * 1e-7 * M_PI / 180.0;  // [deg / 1e-7]->[rad]
  llh[2] = msg->height * 1e-3;              // [mm]->[m]
  double ecef_pos[3];
  llh2xyz(llh, ecef_pos);

  double enu_vel[3] = {msg->velE * 1e-3, msg->velN * 1e-3, -msg->velD * 1e-3};
  double ecef_vel[3];
  enu2xyz_vel(enu_vel, ecef_pos, ecef_vel);

  r.ecef_pos.x = ecef_pos[0];
  r.ecef_pos.y = ecef_pos[1];
  r.ecef_pos.z = ecef_pos[2];
  r.ecef_vel.x = ecef_vel[0];
  r.ecef_vel.y = ecef_vel[1];
  r.ecef_vel.z = ecef_vel[2];

  pub5.publish(r);
}

void gnss_velocity_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  if (nav_msg_ptr == nullptr) return;
  rtklib_msgs::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp = msg->header.stamp;
  double gnss_velocity_time = msg->header.stamp.toSec();
  r.tow = gnss_velocity_time;

  double llh[3];
  llh[0] = nav_msg_ptr->latitude * M_PI / 180.0;
  llh[1] = nav_msg_ptr->longitude * M_PI / 180.0;
  llh[2] = nav_msg_ptr->altitude;
  double ecef_pos[3];
  llh2xyz(llh, ecef_pos);
  r.ecef_pos.x = ecef_pos[0];
  r.ecef_pos.y = ecef_pos[1];
  r.ecef_pos.z = ecef_pos[2];

  r.ecef_vel.x = msg->twist.twist.linear.x;
  r.ecef_vel.y = msg->twist.twist.linear.y;
  r.ecef_vel.z = msg->twist.twist.linear.z;

  pub5.publish(r);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gnss_converter_node");
  ros::NodeHandle n;

  std::string use_gnss_mode;

  std::string node_name = ros::this_node::getName();

  int velocity_source_type = 0;
  // rtklib_msgs/RtklibNav: 0, nmea_msgs/Sentence: 1, ublox_msgs/NavPVT: 2, geometry_msgs/TwistWithCovarianceStamped: 3
  std::string velocity_source_topic;
  int llh_source_type = 0; // rtklib_msgs/RtklibNav: 0, nmea_msgs/Sentence: 1, sensor_msgs/NavSatFix: 2
  std::string llh_source_topic;

  n.getParam("gnss/velocity_source_type",velocity_source_type);
  // n.getParam("gnss/velocity_source_topic",velocity_source_topic);
  n.getParam("gnss/velocity_source_topic",velocity_source_topic);
  n.getParam("gnss/llh_source_type",llh_source_type);
  n.getParam("gnss/llh_source_topic",llh_source_topic);

  std::cout<< "velocity_source_type "<<velocity_source_type<<std::endl;
  std::cout<< "velocity_source_topic "<<velocity_source_topic<<std::endl;
  std::cout<< "llh_source_type "<<llh_source_type<<std::endl;
  std::cout<< "llh_source_topic "<<llh_source_topic<<std::endl;

  ros::Subscriber rtklib_nav_sub, nmea_sentence_sub, navsatfix_sub, navpvt_sub, gnss_velocity_sub;

  if(velocity_source_type == 0)
  {
    rtklib_nav_sub = n.subscribe(velocity_source_topic, 1000, rtklib_nav_callback);
  }
  else if(velocity_source_type == 1)
  {
    nmea_sentence_sub = n.subscribe(velocity_source_topic, 1000, nmea_callback);
  }
  else if(velocity_source_type == 2)
  {
    navpvt_sub = n.subscribe(velocity_source_topic, 1000, navpvt_callback);
  }
  else if(velocity_source_type == 3)
  {
    gnss_velocity_sub = n.subscribe(
        velocity_source_topic, 1000, gnss_velocity_callback);
  }
  else 
  {
    ROS_ERROR("Invalid velocity_source_type");
    ros::shutdown();
  }

  if(llh_source_type == 0)
  {
    rtklib_nav_sub =n.subscribe(llh_source_topic, 1000, rtklib_nav_callback);
  }
  else if(llh_source_type == 1)
  {
    nmea_sentence_sub = n.subscribe(llh_source_topic, 1000, nmea_callback);
  }
  else if(llh_source_type == 2)
  {
    navsatfix_sub = n.subscribe(llh_source_topic, 1000, navsatfix_callback);
  }
  else 
  {
    ROS_ERROR("Invalid llh_source_type");
    ros::shutdown();
  }

  pub1 = n.advertise<sensor_msgs::NavSatFix>("fix", 1000);
  pub2 = n.advertise<nmea_msgs::Gpgga>("gga", 1000);
  pub3 = n.advertise<nmea_msgs::Gprmc>("rmc", 1000);
  pub4 = n.advertise<rtklib_msgs::RtklibNav>("rtklib_nav", 1000);

  ros::spin();

  return 0;
}
