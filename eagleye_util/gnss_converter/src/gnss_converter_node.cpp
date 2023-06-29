
#include "rclcpp/rclcpp.hpp"
#include "gnss_converter/nmea2fix.hpp"
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "eagleye_coordinate/eagleye_coordinate.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub;
rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr gga_pub;
rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr rmc_pub;
rclcpp::Publisher<rtklib_msgs::msg::RtklibNav>::SharedPtr rtklib_nav_pub;


static nmea_msgs::msg::Sentence sentence;
sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_msg_ptr;

static std::string sub_topic_name, pub_fix_topic_name = "fix",
  pub_gga_topic_name = "gga", pub_rmc_topic_name = "rmc" ,pub_rtklib_nav_topic = "rtklib_nav";

void nmea_callback(const nmea_msgs::msg::Sentence::ConstSharedPtr msg)
{
  nmea_msgs::msg::Gpgga gga;
  nmea_msgs::msg::Gprmc rmc;
  sensor_msgs::msg::NavSatFix fix;

  sentence.header = msg->header;
  sentence.sentence = msg->sentence;
  gnss_converter_converter(sentence, &fix, &gga, &rmc);

  rclcpp::Time ros_clock(fix.header.stamp);
  rclcpp::Time ros_clock2(rmc.header.stamp);

  if (ros_clock.seconds() != 0)
  {
    gga.header.frame_id = fix.header.frame_id = "gnss";
    navsatfix_pub->publish(fix);
    gga_pub->publish(gga);
  }
  if (ros_clock2.seconds() != 0)
  {
    rmc.header.frame_id = "gnss";
    rmc_pub->publish(rmc);
  }
}

void rtklib_nav_callback(const rtklib_msgs::msg::RtklibNav::ConstSharedPtr msg) {
  rtklib_nav_pub->publish(*msg);;
}

void navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { nav_msg_ptr = msg; }


void navpvt_callback(const ublox_msgs::msg::NavPVT::ConstSharedPtr msg)
{
  rtklib_msgs::msg::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp.sec = msg->sec;
  r.header.stamp.nanosec = msg->nano;
  if (nav_msg_ptr != nullptr)
    r.status = *nav_msg_ptr;
  r.tow = msg->i_tow;

  double llh[3];
  llh[0] = msg->lat * 1e-7 * M_PI / 180.0;  // [deg / 1e-7]->[rad]
  llh[1] = msg->lon * 1e-7 * M_PI / 180.0;  // [deg / 1e-7]->[rad]
  llh[2] = msg->height * 1e-3;              // [mm]->[m]
  double ecef_pos[3];
  llh2xyz(llh, ecef_pos);

  double enu_vel[3] = {msg->vel_e * 1e-3, msg->vel_n * 1e-3, -msg->vel_d * 1e-3};
  double ecef_vel[3];
  enu2xyz_vel(enu_vel, ecef_pos, ecef_vel);

  r.ecef_pos.x = ecef_pos[0];
  r.ecef_pos.y = ecef_pos[1];
  r.ecef_pos.z = ecef_pos[2];
  r.ecef_vel.x = ecef_vel[0];
  r.ecef_vel.y = ecef_vel[1];
  r.ecef_vel.z = ecef_vel[2];

  rtklib_nav_pub->publish(r);
}

void gnss_velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  if (nav_msg_ptr == nullptr) return;
  rtklib_msgs::msg::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp = msg->header.stamp;
  if (nav_msg_ptr != nullptr)
    r.status = *nav_msg_ptr;
  rclcpp::Time ros_clock(msg->header.stamp);
  double gnss_velocity_time = ros_clock.seconds() * 1e3;
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

  rtklib_nav_pub->publish(r);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gnss_converter_node");

  int velocity_source_type = 0;
  // rtklib_msgs/RtklibNav: 0, nmea_msgs/Sentence: 1, ublox_msgs/NavPVT: 2, geometry_msgs/TwistWithCovarianceStamped: 3
  std::string velocity_source_topic;
  int llh_source_type = 0; // rtklib_msgs/RtklibNav: 0, nmea_msgs/Sentence: 1, sensor_msgs/NavSatFix: 2
  std::string llh_source_topic;

  rclcpp::Subscription<rtklib_msgs::msg::RtklibNav>::SharedPtr rtklib_nav_sub;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_sentence_sub;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr navpvt_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_velocity_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_sub;

  node->declare_parameter("gnss.velocity_source_type",velocity_source_type);
  node->declare_parameter("gnss.velocity_source_topic",velocity_source_topic);
  node->declare_parameter("gnss.llh_source_type",llh_source_type);
  node->declare_parameter("gnss.llh_source_topic",llh_source_topic);
  node->get_parameter("gnss.velocity_source_type",velocity_source_type);
  node->get_parameter("gnss.velocity_source_topic",velocity_source_topic);
  node->get_parameter("gnss.llh_source_type",llh_source_type);
  node->get_parameter("gnss.llh_source_topic",llh_source_topic);

  std::cout<< "velocity_source_type "<<velocity_source_type<<std::endl;
  std::cout<< "velocity_source_topic "<<velocity_source_topic<<std::endl;
  std::cout<< "llh_source_type "<<llh_source_type<<std::endl;
  std::cout<< "llh_source_topic "<<llh_source_topic<<std::endl;
  
  if(velocity_source_type == 0)
  {
    rtklib_nav_sub = node->create_subscription<rtklib_msgs::msg::RtklibNav>(velocity_source_topic, 1000, rtklib_nav_callback);
  }
  else if(velocity_source_type == 1)
  {
    nmea_sentence_sub = node->create_subscription<nmea_msgs::msg::Sentence>(velocity_source_topic, 1000, nmea_callback);
  }
  else if(velocity_source_type == 2)
  {
    navpvt_sub = node->create_subscription<ublox_msgs::msg::NavPVT>(velocity_source_topic, 1000, navpvt_callback);
  }
  else if(velocity_source_type == 3)
  {
    gnss_velocity_sub = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        velocity_source_topic, 1000, gnss_velocity_callback);
  }
  else 
  {
    RCLCPP_ERROR(node->get_logger(),"Invalid velocity_source_type");
    rclcpp::shutdown();
  }

  if(llh_source_type == 0)
  {
    rtklib_nav_sub = node->create_subscription<rtklib_msgs::msg::RtklibNav>(llh_source_topic, 1000, rtklib_nav_callback);
  }
  else if(llh_source_type == 1)
  {
    nmea_sentence_sub = node->create_subscription<nmea_msgs::msg::Sentence>(llh_source_topic, 1000, nmea_callback);
  }
  else if(llh_source_type == 2)
  {
    navsatfix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(llh_source_topic, 1000, navsatfix_callback);
  }
  else 
  {
    RCLCPP_ERROR(node->get_logger(),"Invalid llh_source_type");
    rclcpp::shutdown();
  }

  navsatfix_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>(pub_fix_topic_name, 1000);
  gga_pub = node->create_publisher<nmea_msgs::msg::Gpgga>(pub_gga_topic_name, 1000);
  rmc_pub = node->create_publisher<nmea_msgs::msg::Gprmc>(pub_rmc_topic_name, 1000);
  rtklib_nav_pub = node->create_publisher<rtklib_msgs::msg::RtklibNav>(pub_rtklib_nav_topic, 1000);

  rclcpp::spin(node);

  return 0;
}
