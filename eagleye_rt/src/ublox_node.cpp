#include <eagleye_coordinate/eagleye_coordinate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtklib_msgs/msg/rtklib_nav.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

rclcpp::Publisher<rtklib_msgs::msg::RtklibNav>::SharedPtr pub_rtk;
sensor_msgs::msg::NavSatFix nav_msg;

void fix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { nav_msg = *msg; }

void ublox_callback(const ublox_msgs::msg::NavPVT::ConstSharedPtr msg)
{
  rtklib_msgs::msg::RtklibNav r;
  r.header.frame_id = "gps";
  r.header.stamp.sec = msg->sec;
  r.header.stamp.nanosec = msg->nano;
  r.status = nav_msg;
  r.tow = msg->i_tow;

  double llh[3];
  llh[0] = msg->lat * 1e-7;
  llh[1] = msg->lon * 1e-7;
  llh[2] = msg->height * 1e-3;  // [mm]->[m]
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

  pub_rtk->publish(r);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ublox");

  std::string subscribe_ublox_topic_name = "/sensing/gnss/ublox/navpvt";
  std::string subscribe_fix_topic_name = "/sensing/gnss/ublox/nav_sat_fix";
  std::string publish_rtklib_topic_name = "/rtklib_nav";

  auto sub_ublox = node->create_subscription<ublox_msgs::msg::NavPVT>(subscribe_ublox_topic_name, 1000, ublox_callback);
  auto sub_fix = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_fix_topic_name, 1000, fix_callback);
  pub_rtk = node->create_publisher<rtklib_msgs::msg::RtklibNav>(publish_rtklib_topic_name, 10);

  rclcpp::spin(node);

  return 0;
}
