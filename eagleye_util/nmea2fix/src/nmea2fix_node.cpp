
#include "rclcpp/rclcpp.hpp"
#include "nmea2fix/nmea2fix.hpp"

rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub1;
rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr pub2;
static nmea_msgs::msg::Sentence sentence;

static std::string sub_topic_name,pub_fix_topic_name,pub_gga_topic_name;
static bool output_gga;

void nmea_callback(const nmea_msgs::msg::Sentence::ConstSharedPtr msg)
{
  nmea_msgs::msg::Gpgga gga;
  sensor_msgs::msg::NavSatFix fix;

  rclcpp::Time ros_clock(msg->header.stamp);

  sentence.header = msg->header;
  sentence.sentence = msg->sentence;
  nmea2fix_converter(sentence, &fix, &gga);
  if (ros_clock.seconds() != 0)
  {
    gga.header.frame_id = fix.header.frame_id = "gps";
    pub1->publish(fix);
    if(output_gga) pub2->publish(gga);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nmea2fix_node");

  node->declare_parameter("sub_topic_name",sub_topic_name);
  node->declare_parameter("pub_fix_topic_name",pub_fix_topic_name);
  node->declare_parameter("pub_gga_topic_name",pub_gga_topic_name);
  node->declare_parameter("output_gga",output_gga);

  node->get_parameter("sub_topic_name",sub_topic_name);
  node->get_parameter("pub_fix_topic_name",pub_fix_topic_name);
  node->get_parameter("ub_gga_topic_name",pub_gga_topic_name);
  node->get_parameter("output_gga",output_gga);
  std::cout<< "sub_topic_name "<<sub_topic_name<<std::endl;
  std::cout<< "pub_fix_topic_name "<<pub_fix_topic_name<<std::endl;
  std::cout<< "pub_gga_topic_name "<<pub_gga_topic_name<<std::endl;
  std::cout<< "output_gga "<<output_gga<<std::endl;

  auto sub = node->create_subscription<nmea_msgs::msg::Sentence>(sub_topic_name, 1000, nmea_callback);
  pub1 = node->create_publisher<sensor_msgs::msg::NavSatFix>(pub_fix_topic_name, 1000);
  if(output_gga) pub2 = node->create_publisher<nmea_msgs::msg::Gpgga>(pub_gga_topic_name, 1000);

  rclcpp::spin(node);

  return 0;
}
