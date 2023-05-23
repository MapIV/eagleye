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
 * can_velocity_converter.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "geometry_msgs/TwistStamped.h"

class CANVelocityConverter
{
public:
  CANVelocityConverter()
  {
    nh.getParam("can_id", can_id);
    nh.getParam("start_bit", start_bit);
    nh.getParam("length", length);
    nh.getParam("factor", factor);
    nh.getParam("offset", offset);
    nh.getParam("byte_order", byte_order);
    nh.getParam("value_type", value_type);

    sub = nh.subscribe("/vehicle/can_tx", 1000, &CANVelocityConverter::can_callback, this);
    pub = nh.advertise<geometry_msgs::TwistStamped>("/can_twist", 1000);
  }

  void spin()
  {
    ros::spin();
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  geometry_msgs::TwistStamped msg_velocity;

  int can_id = 0x001;
  int start_bit = 0;
  int length = 16;
  double factor = 0.01;
  double offset = 0.0;
  std::string byte_order = "Motorola";
  std::string value_type = "Unsigned";
  double velocity = 0.0;

  void can_callback(const can_msgs::Frame::ConstPtr& msg)
  {
    unsigned long long int can_sep_data[8];
    unsigned long long int can_data = 0;
    unsigned long long int tmp_unsigned_data;
    unsigned long long int data_mask = pow(2, length) - 1;
    long long int tmp_signed_data;

    if (msg->id == can_id)
    {
      msg_velocity.header.stamp = msg->header.stamp;
      msg_velocity.header.frame_id = "base_link";

      for (int i = 0; i < msg->dlc; i++)
      {
        if (byte_order == "Intel")
        {
          can_sep_data[i] = msg->data[i];
        }
        else if (byte_order == "Motorola")
        {
          can_sep_data[i] = msg->data[(msg->dlc - 1) - i];
        }
        can_data |= can_sep_data[i] << i * 8;
      }

      tmp_unsigned_data = (can_data >> (msg->dlc * 8 - start_bit - length)) & data_mask;

      ROS_INFO("CAN DATA %04llx", can_data);

      if (value_type == "Signed")
      {
        if (tmp_unsigned_data >> length - 1 == 0)
        {
          velocity = tmp_unsigned_data * factor + offset;
        }
        else
        {
          tmp_signed_data = tmp_unsigned_data | (~0 << length);
          velocity = tmp_signed_data * factor + offset;
        }
      }
      else if (value_type == "Unsigned")
      {
        velocity = tmp_unsigned_data * factor + offset;
      }

      msg_velocity.twist.linear.x = velocity / 3.6;

      ROS_INFO("RAW CAN DATA %02x%02x%02x%02x%02x%02x%02x%02x", msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
      ROS_INFO("DATA %04llx", tmp_unsigned_data);
      ROS_INFO("%lf m/s", msg_velocity.twist.linear.x);
      pub.publish(msg_velocity);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_velocity_converter");
  CANVelocityConverter converter;
  converter.spin();
  return 0;
}
