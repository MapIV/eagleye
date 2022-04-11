
#include "rclcpp/rclcpp.hpp"
#include "nmea2fix/nmea2fix.hpp"

rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub1;
rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr pub2;
rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr pub3;
static nmea_msgs::msg::Sentence sentence;

static std::string sub_topic_name, pub_fix_topic_name, pub_gga_topic_name, pub_rmc_topic_name;
static bool output_gga, output_rmc;

void nmea_callback(const nmea_msgs::msg::Sentence::ConstSharedPtr msg)
{
  nmea_msgs::msg::Gpgga gga;
  nmea_msgs::msg::Gprmc rmc;
  sensor_msgs::msg::NavSatFix fix;

  sentence.header = msg->header;
  sentence.sentence = msg->sentence;
  nmea2fix_converter(sentence, &fix, &gga, &rmc);

  rclcpp::Time ros_clock(fix.header.stamp);
  rclcpp::Time ros_clock2(rmc.header.stamp);

  if (ros_clock.seconds() != 0)
  {
    gga.header.frame_id = fix.header.frame_id = "gnss";
    pub1->publish(fix);
    if(output_gga) pub2->publish(gga);
  }
  if (ros_clock2.seconds() != 0 && output_rmc)
  {
    rmc.header.frame_id = "gnss";
    pub3->publish(rmc);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nmea2fix_node");

  std::string use_gnss_mode;

  node->declare_parameter("nmea_sentence_topic",sub_topic_name);
  node->declare_parameter("pub_fix_topic_name",pub_fix_topic_name);
  node->declare_parameter("pub_gga_topic_name",pub_gga_topic_name);
  node->declare_parameter("pub_rmc_topic_name",pub_rmc_topic_name);
  node->declare_parameter("output_gga",output_gga);
  node->declare_parameter("output_rmc",output_rmc);
  node->declare_parameter("use_gnss_mode",use_gnss_mode);

  node->get_parameter("nmea_sentence_topic",sub_topic_name);
  node->get_parameter("pub_fix_topic_name",pub_fix_topic_name);
  node->get_parameter("pub_gga_topic_name",pub_gga_topic_name);
  node->get_parameter("pub_rmc_topic_name",pub_rmc_topic_name);
  node->get_parameter("output_gga",output_gga);
  node->get_parameter("output_rmc",output_rmc);
  node->get_parameter("use_gnss_mode",use_gnss_mode);

  if (use_gnss_mode == "nmea" || use_gnss_mode == "NMEA") // use NMEA mode
  {
    output_gga = true;
    output_rmc = true;
  }

  std::cout<< "sub_topic_name "<<sub_topic_name<<std::endl;
  std::cout<< "pub_fix_topic_name "<<pub_fix_topic_name<<std::endl;
  std::cout<< "pub_gga_topic_name "<<pub_gga_topic_name<<std::endl;
  std::cout<< "pub_rmc_topic_name "<<pub_rmc_topic_name<<std::endl;
  std::cout<< "output_gga "<<output_gga<<std::endl;
  std::cout<< "output_rmc "<<output_rmc<<std::endl;
  std::cout<< "use_gnss_mode "<<use_gnss_mode<<std::endl;

  auto sub = node->create_subscription<nmea_msgs::msg::Sentence>(sub_topic_name, 1000, nmea_callback);
  pub1 = node->create_publisher<sensor_msgs::msg::NavSatFix>(pub_fix_topic_name, 1000);
  if(output_gga) pub2 = node->create_publisher<nmea_msgs::msg::Gpgga>(pub_gga_topic_name, 1000);
  if(output_rmc) pub3 = node->create_publisher<nmea_msgs::msg::Gprmc>(pub_rmc_topic_name, 1000);

  rclcpp::spin(node);

  return 0;
}
