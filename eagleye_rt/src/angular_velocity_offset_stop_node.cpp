// Copyright (c) 2019, Map IV, Inc.
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
 * angular_velocity_offset_stop.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "navigation/angular_velocity_offset_stop.hpp"
#include "navigation/navigation.hpp"

class AngularVelocityOffsetStopNode
{
public:
  AngularVelocityOffsetStopNode(ros::NodeHandle& nh) : nh_(nh)
  {
    std::string yaml_file;
    nh_.getParam("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    AngularVelocityOffsetStopParameter param;
    param.load(yaml_file);
    estimator_.setParameter(param);

    offset_stop_pub_ = nh_.advertise<eagleye_msgs::AngularVelocityOffset>("angular_velocity_offset_stop", 1000);
    twist_sub_ = nh_.subscribe("vehicle/twist", 1000, &AngularVelocityOffsetStopNode::twistCallback, this);
    imu_sub_ = nh_.subscribe("imu/data_tf_converted", 1000, &AngularVelocityOffsetStopNode::imuCallback, this);
  }
  void run()
  {
    ros::spin();
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher offset_stop_pub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber imu_sub_;

  // Estimator
  AngularVelocityOffsetStopEstimator estimator_;

  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    Eigen::Vector3d linear_velocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    estimator_.velocityCallback(linear_velocity);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // Trigger estimation
    Eigen::Vector3d angular_velocity(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    AngularVelocityOffsetStopStatus offset_stop_status = estimator_.imuCallback(angular_velocity);
    
    // Convert to ROS message
    eagleye_msgs::AngularVelocityOffset offset_stop_msg;
    offset_stop_msg.header = msg->header;
    offset_stop_msg.angular_velocity_offset.x = offset_stop_status.offset_stop[0];
    offset_stop_msg.angular_velocity_offset.y = offset_stop_status.offset_stop[1];
    offset_stop_msg.angular_velocity_offset.z = offset_stop_status.offset_stop[2];
    offset_stop_msg.status.estimate_status = offset_stop_status.is_estimated_now;
    offset_stop_msg.status.enabled_status = offset_stop_status.is_estimation_started;

    // Publish
    offset_stop_pub_.publish(offset_stop_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "angular_velocity_offset_stop");
  ros::NodeHandle nh;

  AngularVelocityOffsetStopNode node(nh);
  node.run();

  return 0;
}
