
#include "ros/ros.h"
#include "nmea2fix/nmea2fix.hpp"

static ros::Publisher pub1,pub2, pub3;
static nmea_msgs::Sentence sentence;

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
    if(output_gga) pub2.publish(gga);
  }
  if (rmc.header.stamp.toSec() != 0 && output_rmc)
  {
    rmc.header.frame_id = "gnss";
    pub3.publish(rmc);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmea2fix_node");
  ros::NodeHandle n;

  std::string use_gnss_mode;

  n.getParam("nmea_sentence_topic",sub_topic_name);
  n.getParam("nmea2fix_node/pub_fix_topic_name",pub_fix_topic_name);
  n.getParam("nmea2fix_node/pub_gga_topic_name",pub_gga_topic_name);
  n.getParam("nmea2fix_node/pub_rmc_topic_name",pub_rmc_topic_name);
  n.getParam("nmea2fix_node/output_gga",output_gga);
  n.getParam("nmea2fix_node/output_rmc",output_rmc);
  n.getParam("eagleye/use_gnss_mode",use_gnss_mode);

  if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
  {
    output_gga = true;
    output_rmc = true;
  }

  std::cout<< "sub_topic_name "<<sub_topic_name<<std::endl;
  std::cout<< "pub_fix_topic_name "<<pub_fix_topic_name<<std::endl;
  std::cout<< "pub_gga_topic_name "<<pub_gga_topic_name<<std::endl;
  std::cout<< "pub_rmc_topic_name "<<pub_rmc_topic_name<<std::endl;
  std::cout<< "use_gnss_mode "<<use_gnss_mode<<std::endl;
  std::cout<< "output_gga "<<output_gga<<std::endl;
  std::cout<< "output_rmc "<<output_rmc<<std::endl;

  ros::Subscriber sub = n.subscribe(sub_topic_name, 1000, nmea_callback);
  pub1 = n.advertise<sensor_msgs::NavSatFix>(pub_fix_topic_name, 1000);
  if(output_gga) pub2 = n.advertise<nmea_msgs::Gpgga>(pub_gga_topic_name, 1000);
  if(output_rmc) pub3 = n.advertise<nmea_msgs::Gprmc>(pub_rmc_topic_name, 1000);

  ros::spin();

  return 0;
}
