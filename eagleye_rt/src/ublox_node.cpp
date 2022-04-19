#include <eagleye_coordinate/eagleye_coordinate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtklib_msgs/msg/rtklib_nav.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

rclcpp::Publisher<rtklib_msgs::msg::RtklibNav>::SharedPtr pub_rtk;

void ublox_callback(const ublox_msgs::msg::NavPVT::ConstSharedPtr msg)
{
  rtklib_msgs::msg::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp.sec = msg->sec;
  r.header.stamp.nanosec = msg->nano;
  // r.status;            // sensor_msgs::msg::NavSatFix // これはmoniorに出力されるだけ
  r.tow = msg->i_tow;  // int64_t<-uint32_t NOTE:

  double llh[3];
  llh[0] = msg->lat * 1e-7;
  llh[1] = msg->lon * 1e-7;
  llh[2] = msg->height;
  double xyz[3];
  llh2xyz(llh, xyz);

  r.ecef_pos.x = xyz[0];
  r.ecef_pos.y = xyz[1];
  r.ecef_pos.z = xyz[2];
  r.ecef_vel.x = msg->vel_e * 1e-3;
  r.ecef_vel.y = msg->vel_n * 1e-3;
  r.ecef_vel.z = -msg->vel_d * 1e-3;

  pub_rtk->publish(r);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ublox");

  std::string subscribe_ublox_topic_name = "/sensing/gnss/ublox/navpvt";
  std::string publish_rtklib_topic_name = "/rtklib_nav";

  auto sub = node->create_subscription<ublox_msgs::msg::NavPVT>(subscribe_ublox_topic_name, 1000, ublox_callback);
  pub_rtk = node->create_publisher<rtklib_msgs::msg::RtklibNav>(publish_rtklib_topic_name, 10);

  rclcpp::spin(node);

  return 0;
}
