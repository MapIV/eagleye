
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "eagleye_msgs/Heading.h"
#include "eagleye_msgs/Position.h"
#include "rtklib_msgs/RtklibNav.h"
#include "tf/transform_broadcaster.h"
#include "xyz2enu.hpp"
#include "enu2llh.hpp"
#include "hgeoid.hpp"

static eagleye_msgs::Heading eagleye_heading;
static eagleye_msgs::Position eagleye_position;
static geometry_msgs::Quaternion _quat;
static sensor_msgs::NavSatFix eagleye_fix;

static ros::Publisher pub;
static geometry_msgs::PoseStamped pose;

static double m_PLo, m_PLato;
static double m_lat,m_lon,m_h;
static double m_x,m_y,m_z;
static int _plane = 7;

static rtklib_msgs::RtklibNav rtklib_nav;
static eagleye_msgs::Position enu_absolute_pos;
static double enu_pos[3];
static double llh_pos[3];
static bool specify_base_pos = false;
static double ecef_base_pos_x = 0;
static double ecef_base_pos_y = 0;
static double ecef_base_pos_z = 0;


void eagleye_heading_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  eagleye_heading.header = msg->header;
  eagleye_heading.heading_angle = msg->heading_angle;
  eagleye_heading.status = msg->status;
}

void eagleye_enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
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
  double u_pos;

  ecef_pos[0] = msg->ecef_pos.x;
  ecef_pos[1] = msg->ecef_pos.y;
  ecef_pos[2] = msg->ecef_pos.z;
  ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
  ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
  ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;

  xyz2enu(ecef_pos, ecef_base_pos, enu_pos);
  enu2llh(enu_pos, ecef_base_pos, llh_pos);

}

void set_plane(int num)
{
  int lon_deg, lon_min, lat_deg, lat_min; // longitude and latitude of origin of each plane in Japan
  if (num == 1)
  {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 129;
    lat_min = 30;
  }
  else if (num == 2)
  {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 131;
    lat_min = 0;
  }
  else if (num == 3)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 132;
    lat_min = 10;
  }
  else if (num == 4)
  {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 133;
    lat_min = 30;
  }
  else if (num == 5)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 134;
    lat_min = 20;
  }
  else if (num == 6)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 136;
    lat_min = 0;
  }
  else if (num == 7)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 137;
    lat_min = 10;
  }
  else if (num == 8)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 138;
    lat_min = 30;
  }
  else if (num == 9)
  {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 139;
    lat_min = 50;
  }
  else if (num == 10)
  {
    lon_deg = 40;
    lon_min = 0;
    lat_deg = 140;
    lat_min = 50;
  }
  else if (num == 11)
  {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 140;
    lat_min = 15;
  }
  else if (num == 12)
  {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 142;
    lat_min = 15;
  }
  else if (num == 13)
  {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 144;
    lat_min = 15;
  }
  else if (num == 14)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 142;
    lat_min = 0;
  }
  else if (num == 15)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 127;
    lat_min = 30;
  }
  else if (num == 16)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 124;
    lat_min = 0;
  }
  else if (num == 17)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 131;
    lat_min = 0;
  }
  else if (num == 18)
  {
    lon_deg = 20;
    lon_min = 0;
    lat_deg = 136;
    lat_min = 0;
  }
  else if (num == 19)
  {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 154;
    lat_min = 0;
  }

  // swap longitude and latitude
  m_PLo = M_PI * ((double)lat_deg + (double)lat_min / 60.0) / 180.0;
  m_PLato = M_PI * ((double)lon_deg + (double)lon_min / 60.0) / 180;
}

void conv_llh2xyz(void)
{
  double PS;   //
  double PSo;  //
  double PDL;  //
  double Pt;   //
  double PN;   //
  double PW;   //

  double PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9;
  double PA, PB, PC, PD, PE, PF, PG, PH, PI;
  double Pe;   //
  double Pet;  //
  double Pnn;  //
  double AW, FW, Pmo;

  Pmo = 0.9999;

  /*WGS84 Parameters*/
  AW = 6378137.0;            // Semimajor Axis
  FW = 1.0 / 298.257222101;  // 298.257223563 //Geometrical flattening

  Pe = (double)sqrt(2.0 * FW - pow(FW, 2));
  Pet = (double)sqrt(pow(Pe, 2) / (1.0 - pow(Pe, 2)));

  PA = (double)1.0 + 3.0 / 4.0 * pow(Pe, 2) + 45.0 / 64.0 * pow(Pe, 4) + 175.0 / 256.0 * pow(Pe, 6) +
       11025.0 / 16384.0 * pow(Pe, 8) + 43659.0 / 65536.0 * pow(Pe, 10) + 693693.0 / 1048576.0 * pow(Pe, 12) +
       19324305.0 / 29360128.0 * pow(Pe, 14) + 4927697775.0 / 7516192768.0 * pow(Pe, 16);

  PB = (double)3.0 / 4.0 * pow(Pe, 2) + 15.0 / 16.0 * pow(Pe, 4) + 525.0 / 512.0 * pow(Pe, 6) +
       2205.0 / 2048.0 * pow(Pe, 8) + 72765.0 / 65536.0 * pow(Pe, 10) + 297297.0 / 262144.0 * pow(Pe, 12) +
       135270135.0 / 117440512.0 * pow(Pe, 14) + 547521975.0 / 469762048.0 * pow(Pe, 16);

  PC = (double)15.0 / 64.0 * pow(Pe, 4) + 105.0 / 256.0 * pow(Pe, 6) + 2205.0 / 4096.0 * pow(Pe, 8) +
       10395.0 / 16384.0 * pow(Pe, 10) + 1486485.0 / 2097152.0 * pow(Pe, 12) + 45090045.0 / 58720256.0 * pow(Pe, 14) +
       766530765.0 / 939524096.0 * pow(Pe, 16);

  PD = (double)35.0 / 512.0 * pow(Pe, 6) + 315.0 / 2048.0 * pow(Pe, 8) + 31185.0 / 131072.0 * pow(Pe, 10) +
       165165.0 / 524288.0 * pow(Pe, 12) + 45090045.0 / 117440512.0 * pow(Pe, 14) +
       209053845.0 / 469762048.0 * pow(Pe, 16);

  PE = (double)315.0 / 16384.0 * pow(Pe, 8) + 3465.0 / 65536.0 * pow(Pe, 10) + 99099.0 / 1048576.0 * pow(Pe, 12) +
       4099095.0 / 29360128.0 * pow(Pe, 14) + 348423075.0 / 1879048192.0 * pow(Pe, 16);

  PF = (double)693.0 / 131072.0 * pow(Pe, 10) + 9009.0 / 524288.0 * pow(Pe, 12) +
       4099095.0 / 117440512.0 * pow(Pe, 14) + 26801775.0 / 469762048.0 * pow(Pe, 16);

  PG = (double)3003.0 / 2097152.0 * pow(Pe, 12) + 315315.0 / 58720256.0 * pow(Pe, 14) +
       11486475.0 / 939524096.0 * pow(Pe, 16);

  PH = (double)45045.0 / 117440512.0 * pow(Pe, 14) + 765765.0 / 469762048.0 * pow(Pe, 16);

  PI = (double)765765.0 / 7516192768.0 * pow(Pe, 16);

  PB1 = (double)AW * (1.0 - pow(Pe, 2)) * PA;
  PB2 = (double)AW * (1.0 - pow(Pe, 2)) * PB / -2.0;
  PB3 = (double)AW * (1.0 - pow(Pe, 2)) * PC / 4.0;
  PB4 = (double)AW * (1.0 - pow(Pe, 2)) * PD / -6.0;
  PB5 = (double)AW * (1.0 - pow(Pe, 2)) * PE / 8.0;
  PB6 = (double)AW * (1.0 - pow(Pe, 2)) * PF / -10.0;
  PB7 = (double)AW * (1.0 - pow(Pe, 2)) * PG / 12.0;
  PB8 = (double)AW * (1.0 - pow(Pe, 2)) * PH / -14.0;
  PB9 = (double)AW * (1.0 - pow(Pe, 2)) * PI / 16.0;

  PS = (double)PB1 * m_lat + PB2 * sin(2.0 * m_lat) + PB3 * sin(4.0 * m_lat) + PB4 * sin(6.0 * m_lat) +
       PB5 * sin(8.0 * m_lat) + PB6 * sin(10.0 * m_lat) + PB7 * sin(12.0 * m_lat) + PB8 * sin(14.0 * m_lat) +
       PB9 * sin(16.0 * m_lat);

  PSo = (double)PB1 * m_PLato + PB2 * sin(2.0 * m_PLato) + PB3 * sin(4.0 * m_PLato) + PB4 * sin(6.0 * m_PLato) +
        PB5 * sin(8.0 * m_PLato) + PB6 * sin(10.0 * m_PLato) + PB7 * sin(12.0 * m_PLato) + PB8 * sin(14.0 * m_PLato) +
        PB9 * sin(16.0 * m_PLato);

  PDL = (double)m_lon - m_PLo;
  Pt = (double)tan(m_lat);
  PW = (double)sqrt(1.0 - pow(Pe, 2) * pow(sin(m_lat), 2));
  PN = (double)AW / PW;
  Pnn = (double)sqrt(pow(Pet, 2) * pow(cos(m_lat), 2));

  m_x = (double)((PS - PSo) + (1.0 / 2.0) * PN * pow(cos(m_lat), 2.0) * Pt * pow(PDL, 2.0) +
                 (1.0 / 24.0) * PN * pow(cos(m_lat), 4) * Pt *
                     (5.0 - pow(Pt, 2) + 9.0 * pow(Pnn, 2) + 4.0 * pow(Pnn, 4)) * pow(PDL, 4) -
                 (1.0 / 720.0) * PN * pow(cos(m_lat), 6) * Pt *
                     (-61.0 + 58.0 * pow(Pt, 2) - pow(Pt, 4) - 270.0 * pow(Pnn, 2) + 330.0 * pow(Pt, 2) * pow(Pnn, 2)) *
                     pow(PDL, 6) -
                 (1.0 / 40320.0) * PN * pow(cos(m_lat), 8) * Pt *
                     (-1385.0 + 3111 * pow(Pt, 2) - 543 * pow(Pt, 4) + pow(Pt, 6)) * pow(PDL, 8)) *
        Pmo;

  m_y = (double)(PN * cos(m_lat) * PDL -
                 1.0 / 6.0 * PN * pow(cos(m_lat), 3) * (-1 + pow(Pt, 2) - pow(Pnn, 2)) * pow(PDL, 3) -
                 1.0 / 120.0 * PN * pow(cos(m_lat), 5) *
                     (-5.0 + 18.0 * pow(Pt, 2) - pow(Pt, 4) - 14.0 * pow(Pnn, 2) + 58.0 * pow(Pt, 2) * pow(Pnn, 2)) *
                     pow(PDL, 5) -
                 1.0 / 5040.0 * PN * pow(cos(m_lat), 7) *
                     (-61.0 + 479.0 * pow(Pt, 2) - 179.0 * pow(Pt, 4) + pow(Pt, 6)) * pow(PDL, 7)) *
        Pmo;

  m_z = m_h;
}

void eagleye_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  double llh[3],hei;

  llh[0] = msg->latitude;
  llh[1] = msg->longitude;
  llh[2] = msg->altitude;

  m_lat = msg->latitude * M_PI / 180;
  m_lon = msg->longitude * M_PI / 180;
  m_h = msg->altitude;

  hgeoid(llh,&hei);

  ROS_INFO("h = %lf",hei);

  m_h = m_h - hei;

  set_plane(_plane);
  conv_llh2xyz();

  eagleye_heading.heading_angle = fmod(eagleye_heading.heading_angle,2*M_PI);
  _quat = tf::createQuaternionMsgFromYaw((90* M_PI / 180)-eagleye_heading.heading_angle);

  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose.position.x = m_y;
  pose.pose.position.y = m_x;
  pose.pose.position.z = m_z;
  pose.pose.orientation = _quat;
  pub.publish(pose);



  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180)-eagleye_heading.heading_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "eagleye1"));


  //RTKLIB pose
  m_lat = llh_pos[1]* M_PI / 180;
  m_lon = llh_pos[0]* M_PI / 180;
  m_h = llh_pos[2];
  m_h = m_h - hei;


  set_plane(_plane);
  conv_llh2xyz();

  pose.pose.position.x = m_y;
  pose.pose.position.y = m_x;
  pose.pose.position.z = m_z;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  q.setRPY(0, 0, (90* M_PI / 180)-eagleye_heading.heading_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "rtklib1"));

  //ROS_INFO("m_x = %lf, m_y = %lf, m_z = %lf",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fix2pose");
  ros::NodeHandle n;

  n.getParam("/eagleye/position/specify_base_pos",specify_base_pos);
  n.getParam("/eagleye/position/ecef_base_pos_x",ecef_base_pos_x);
  n.getParam("/eagleye/position/ecef_base_pos_y",ecef_base_pos_y);
  n.getParam("/eagleye/position/ecef_base_pos_z",ecef_base_pos_z);

  std::cout<< "specify_base_pos "<<specify_base_pos<<std::endl;
  std::cout<< "ecef_base_pos_x "<<ecef_base_pos_x<<std::endl;
  std::cout<< "ecef_base_pos_y "<<ecef_base_pos_y<<std::endl;
  std::cout<< "ecef_base_pos_z "<<ecef_base_pos_z<<std::endl;

  ros::Subscriber sub1 = n.subscribe("/eagleye/heading_interpolate_3rd", 1000, eagleye_heading_callback);
  ros::Subscriber sub2 = n.subscribe("/eagleye/enu_absolute_pos_interpolate", 1000, eagleye_enu_absolute_pos_callback);
  ros::Subscriber sub3 = n.subscribe("/eagleye/fix", 1000, eagleye_fix_callback);
  ros::Subscriber sub4 = n.subscribe("/rtklib_nav", 1000, rtklib_nav_callback);
  //pub = n.advertise<geometry_msgs::PoseStamped>("/eagleye/eagleye_pose", 1000);
  pub = n.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  ros::spin();

  return 0;
}
