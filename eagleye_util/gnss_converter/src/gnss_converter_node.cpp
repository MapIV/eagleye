#include "ros/ros.h"
#include "gnss_converter/nmea2fix.hpp"
#include "rtklib_msgs/RtklibNav.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <coordinate/coordinate.hpp>
#include <ublox_msgs/NavPVT.h>
#include <yaml-cpp/yaml.h>

class GnssConverterNode
{
public:
  GnssConverterNode()
  {
    // Initialization
    ros::NodeHandle n;

    // Setup parameters
    setupParameters(n);

    // Setup publishers
    pub1 = n.advertise<sensor_msgs::NavSatFix>("fix", 1000);
    pub2 = n.advertise<nmea_msgs::Gpgga>("gga", 1000);
    pub3 = n.advertise<nmea_msgs::Gprmc>("rmc", 1000);
    pub4 = n.advertise<rtklib_msgs::RtklibNav>("rtklib_nav", 1000);

    if(!is_sub_antenna)
    {
      // Setup subscribers
      if (velocity_source_type == 0)
      {
        rtklib_nav_sub = n.subscribe(velocity_source_topic, 1000, &GnssConverterNode::rtklib_nav_callback, this);
      }
      else if (velocity_source_type == 1)
      {
        nmea_sentence_sub = n.subscribe(velocity_source_topic, 1000, &GnssConverterNode::nmea_callback, this);
      }
      else if (velocity_source_type == 2)
      {
        navpvt_sub = n.subscribe(velocity_source_topic, 1000, &GnssConverterNode::navpvt_callback, this);
      }
      else if (velocity_source_type == 3)
      {
        gnss_velocity_sub = n.subscribe(velocity_source_topic, 1000, &GnssConverterNode::gnss_velocity_callback, this);
      }
      else
      {
        ROS_ERROR("Invalid velocity_source_type");
        ros::shutdown();
      }
    }

    if (llh_source_type == 0)
    {
      rtklib_nav_sub = n.subscribe(llh_source_topic, 1000, &GnssConverterNode::rtklib_nav_callback, this);
    }
    else if (llh_source_type == 1)
    {
      nmea_sentence_sub = n.subscribe(llh_source_topic, 1000, &GnssConverterNode::nmea_callback, this);
    }
    else if (llh_source_type == 2)
    {
      navsatfix_sub = n.subscribe(llh_source_topic, 1000, &GnssConverterNode::navsatfix_callback, this);
    }
    else
    {
      ROS_ERROR("Invalid llh_source_type");
      ros::shutdown();
    }
  }

  void spin()
  {
    ros::spin();
  }

private:
  void setupParameters(ros::NodeHandle &n)
  {
    std::string node_namespace = ros::this_node::getName();

    n.getParam(node_namespace + "/is_sub_antenna", is_sub_antenna);
    std::cout << node_namespace + "/is_sub_antenna " << is_sub_antenna << std::endl;
    std::string yaml_file;
    n.getParam(node_namespace + "/yaml_file",yaml_file);
    std::cout << "yaml_file " << yaml_file << std::endl;
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);
      if(!is_sub_antenna)
      {
        n.getParam("twist_covariance_thresh", twist_covariance_thresh);
        n.getParam("ublox_vacc_thresh", ublox_vacc_thresh);
        llh_source_type = conf["gnss"]["llh_source_type"].as<int>();
        llh_source_topic = conf["gnss"]["llh_source_topic"].as<std::string>();
        velocity_source_type = conf["gnss"]["velocity_source_type"].as<int>();
        velocity_source_topic = conf["gnss"]["velocity_source_topic"].as<std::string>();
        std::cout << "velocity_source_type " << velocity_source_type << std::endl;
        std::cout << "velocity_source_topic " << velocity_source_topic << std::endl;
        std::cout << "twist_covariance_thresh " << twist_covariance_thresh << std::endl;
        std::cout << "ublox_vacc_thresh " << ublox_vacc_thresh << std::endl;
      }
      else
      {
        llh_source_type = conf["sub_gnss"]["llh_source_type"].as<int>();
        llh_source_topic = conf["sub_gnss"]["llh_source_topic"].as<std::string>();
      }
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mgnss_converter Node YAML Error: " << e.msg << "\033[0m" << std::endl;
      exit(3);
    }

    
    std::cout << "llh_source_type " << llh_source_type << std::endl;
    std::cout << "llh_source_topic " << llh_source_topic << std::endl;
    std::cout << "is_sub_antenna " << is_sub_antenna << std::endl;
  }

  // Callback functions
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
  if(msg->sAcc > ublox_vacc_thresh) return;
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

  pub4.publish(r);
}

void gnss_velocity_callback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  if(msg->twist.covariance[0] > twist_covariance_thresh) return;
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

  pub4.publish(r);
}

  // Member variables
  ros::Publisher pub1, pub2, pub3, pub4;
  ros::Subscriber rtklib_nav_sub, nmea_sentence_sub, navsatfix_sub, navpvt_sub, gnss_velocity_sub;
  nmea_msgs::Sentence sentence;
  sensor_msgs::NavSatFix::ConstPtr nav_msg_ptr;
  int velocity_source_type, llh_source_type;
  std::string velocity_source_topic, llh_source_topic;

  double twist_covariance_thresh = 0.2;
  double ublox_vacc_thresh = 200.0;
  bool is_sub_antenna = false;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_converter_node");

  GnssConverterNode gnss_converter_node;

  gnss_converter_node.spin();

  return 0;
}
