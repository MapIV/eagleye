
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nmea_msgs/Sentence.h"
#include "coordinate.hpp"
#include <string>
#include <memory>

static ros::Publisher pub;
static sensor_msgs::NavSatFix fix;

static double m_lat,m_lon,m_h,undulation;
static int gps_quality;

bool altitude_estimate = true;

double roll_, pitch_, yaw_;
double orientation_time_, position_time_;
ros::Time current_time_, orientation_stamp_;


std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr1,str_vec_ptr2;
  std::string token1,token2,token;
  std::stringstream ss(string);
  int i;
  int index_length;

  while (getline(ss, token1, '\n'))
    str_vec_ptr1.push_back(token1);

  index_length = std::distance(str_vec_ptr1.begin(), str_vec_ptr1.end());

  for (i = 0; i < index_length; i++)
  {
    if (str_vec_ptr1[i].compare(0, 6, "$GNGGA") ==0)
    {
      token = str_vec_ptr1[i];
      break;
    }
    else if (str_vec_ptr1[i].compare(0, 6, "$GNRMC") ==0)
    {
      token = str_vec_ptr1[i];
    }
  }

  std::stringstream ss1(token);

  while (getline(ss1, token2, ','))
    str_vec_ptr2.push_back(token2);

  return str_vec_ptr2;
}

void convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      orientation_stamp_ = current_stamp;
      ROS_INFO("QQ is subscribed.");
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      ROS_INFO("PASHR is subscribed.");
    }
    else if(nmea.at(0) == "$GNGGA")
    {
      position_time_ = stod(nmea.at(1));
      m_lat = stod(nmea.at(2));
      m_lon = stod(nmea.at(4));
      gps_quality = stod(nmea.at(6));
      m_h = stod(nmea.at(9));
      undulation =  stod(nmea.at(11));
      ROS_INFO("GGA is subscribed.");
    }
    else if(nmea.at(0) == "$GNRMC")
    {
      position_time_ = stoi(nmea.at(1));
      m_lat = stod(nmea.at(3));
      m_lon = stod(nmea.at(5));
      m_h = 0.0;
      ROS_INFO("GPRMC is subscribed.");
    }
  }catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void nmea_callback(const nmea_msgs::Sentence::ConstPtr &msg)
{

  // double llh[3] = {0};
  // double _llh[3] = {0};
  // double xyz[3] = {0};
  // double height;

  convert(split(msg->sentence), msg->header.stamp);

  if (gps_quality == 4)
  {
    fix.status.status = 0;
  }
  else
  {
    fix.status.status = -1;
  }

  // llh[0] = (std::floor(m_lat/100) + (m_lat/100 - std::floor(m_lat/100))*100/60) * M_PI / 180;
  // llh[1] = (std::floor(m_lon/100) + (m_lon/100 - std::floor(m_lon/100))*100/60) * M_PI / 180;
  // llh[2] = m_h;
  //
  // if (altitude_estimate == true)
  // {
  //   _llh[0] = llh[0] * 180/M_PI;
  //   _llh[1] = llh[1] * 180/M_PI;
  //   _llh[2] = llh[2];
  //   hgeoid(_llh,&height);
  //   llh[2] = llh[2] - height;
  // }

  fix.header = msg->header;
  fix.header.frame_id = "gps";
  fix.latitude = (std::floor(m_lat/100) + (m_lat/100 - std::floor(m_lat/100))*100/60);
  fix.longitude = (std::floor(m_lon/100) + (m_lon/100 - std::floor(m_lon/100))*100/60);
  fix.altitude = m_h;
  pub.publish(fix);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmea2fix");
  ros::NodeHandle n;

  // n.getParam("plane",plane);
  // n.getParam("altitude_estimate",altitude_estimate);
  // std::cout<< "plane "<<plane<<std::endl;
  // std::cout<< "altitude_estimate "<<altitude_estimate<<std::endl;

  ros::Subscriber sub = n.subscribe("/nmea/sentence", 1000, nmea_callback);
  pub = n.advertise<sensor_msgs::NavSatFix>("f9p/fix", 1000);
  ros::spin();

  return 0;
}
