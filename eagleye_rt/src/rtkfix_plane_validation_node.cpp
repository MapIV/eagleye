// Copyright (c) 2023, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * rtkfix_plane_validation_node.cpp
 * Author MapIV Takanose
 */

#include "rclcpp/rclcpp.hpp"
#include "eagleye_navigation/rtkfix_plane_validation.hpp"
#include "eagleye_navigation/eagleye_navigation.hpp"

class RtkfixPlaneValidationNode
{
public:
  RtkfixPlaneValidationNode(rclcpp::Node::SharedPtr node) : node_(node)
  {
    std::string yaml_file;
    node_->declare_parameter("yaml_file",yaml_file);
    node_->get_parameter("yaml_file",yaml_file);
    // std::cout << "yaml_file: " << yaml_file << std::endl;

    std::string subscribe_gga_topic_name = "gnss/gga";
    std::string subscribe_fix_topic_name = "gnss/fix";

    RtkfixPlaneValidationParameter param;
    param.load(yaml_file);
    estimator_.setParameter(param);

    reliable_rtkfix_pub_ = node->create_publisher<sensor_msgs::msg::NavSatFix>("gnss/reliable_rtkfix", 1000);
    // gga_sub_ = node->create_subscription<nmea_msgs::msg::Gpgga>(subscribe_gga_topic_name, 1000, std::bind(&RtkfixPlaneValidationNode::ggaCallback, this, std::placeholders::_1));
    fix_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(subscribe_fix_topic_name, 1000, std::bind(&RtkfixPlaneValidationNode::fixCallback, this, std::placeholders::_1));
    distance_sub_ = node->create_subscription<eagleye_msgs::msg::Distance>("distance", 1000, std::bind(&RtkfixPlaneValidationNode::distanceCallback, this, std::placeholders::_1));
    enu_vel_sub_ = node->create_subscription<geometry_msgs::msg::Vector3Stamped>("enu_vel", 1000, std::bind(&RtkfixPlaneValidationNode::enuvelCallback, this, std::placeholders::_1));
  }
  void run()
  {
    rclcpp::spin(node_);
  }

private:
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr reliable_rtkfix_pub_;
  rclcpp::Subscription<nmea_msgs::msg::Gpgga>::SharedPtr gga_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<eagleye_msgs::msg::Distance>::SharedPtr distance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr enu_vel_sub_;

  nmea_msgs::msg::Gpgga gga_;
  eagleye_msgs::msg::Distance distance_;
  geometry_msgs::msg::Vector3Stamped enu_vel_;

  bool is_gga_ready_ = false;
  bool is_distance_ready_ = false;

  // Estimator
  RtkfixPlaneValidationEstimator estimator_;

  void fixCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
  {
    // fix2gga
    nmea_msgs::msg::Gpgga gga;
    gga.header = msg->header;
    // gga.utc_seconds = msg->header.stamp.seconds();
    gga.lat = msg->latitude;
    gga.lon = msg->longitude;
    gga.lat_dir = msg->latitude > 0 ? "N" : "S";
    gga.lon_dir = msg->longitude > 0 ? "E" : "W";

    gga.num_sats = msg->status.service;
    gga.hdop = msg->position_covariance[0];
    gga.alt = msg->altitude;
    gga.altitude_units = "M";
    gga.undulation = 0;
    gga.undulation_units = "M";
    gga.diff_age = 0;
    gga.station_id = "";

    double std_pos = 0.1; // [m]
    if (msg->position_covariance[0] < std_pos * std_pos)
    {
      gga.gps_qual = 4;
    }
    else
    {
      gga.gps_qual = 0;
    }

    gga_ = gga;
    is_gga_ready_ = true;
  }

  void ggaCallback(const nmea_msgs::msg::Gpgga::ConstSharedPtr msg)
  {
    gga_ = *msg;
    is_gga_ready_ = true;

    // rclcpp::Time ros_clock(gga_.header.stamp);
    // double time = ros_clock.seconds();
    // std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    // std::cout << "***debug-code: gga header: " << time << std::endl; 
  }

  void distanceCallback(const eagleye_msgs::msg::Distance::ConstSharedPtr msg)
  {
    distance_ = *msg;
    is_distance_ready_ = true;

    // rclcpp::Time ros_clock(distance_.header.stamp);
    // double time = ros_clock.seconds();
    // std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    // std::cout << "***debug-code: distance header: " << time << std::endl; 
  }

  void enuvelCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    // Skip until msgs are ready
    if (!is_gga_ready_) return;
    if (!is_distance_ready_) return;

    enu_vel_ = *msg;

    // rclcpp::Time ros_clock(enu_vel_.header.stamp);
    // double time = ros_clock.seconds();
    // std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    // std::cout << "***debug-code: enu_vel header: " << time << std::endl; 

    // // Trigger estimation
    RtkfixPlaneValidationStatus reliable_rtkfix_status = estimator_.estimate(gga_, distance_, enu_vel_);

    if(reliable_rtkfix_status.is_estimation_reliable)
    {
      // Convert to ROS message
      sensor_msgs::msg::NavSatFix reliable_rtkfix_msg;
      reliable_rtkfix_msg = reliable_rtkfix_status.fix_msg;

      // Publish
      reliable_rtkfix_pub_->publish(reliable_rtkfix_msg);
    }
  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  RtkfixPlaneValidationNode node(rclcpp::Node::make_shared("rtkfix_plane_validation"));
  node.run();

  return 0;

}
