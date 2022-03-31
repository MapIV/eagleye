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

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub; 
static geometry_msgs::msg::TwistStamped msg_velocity;

static int can_id = 0x001;
static int start_bit = 0;
static int length = 16;
static double factor = 0.01;
static double offset = 0.0;
static std::string byte_order = "Motorola";
static std::string value_type = "Unsigned";
static double velocity = 0.0;
static std::string node_name = "can_velocity_converter";

void can_callback(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  unsigned long long int can_sep_data[8];
  unsigned long long int can_data, tmp_unsigned_data;
  unsigned long long int data_mask = pow(2, length) - 1;
  long long int tmp_signed_data;

  if(msg->id == can_id){
    msg_velocity.header.stamp = msg->header.stamp;
    msg_velocity.header.frame_id = "base_link";

    for (int i = 0; i < msg->dlc; i++)
    {
      if(byte_order == "Intel")
      {
        can_sep_data[i] = msg->data[i];
      }
      else if(byte_order == "Motorola")
      {
        can_sep_data[i] = msg->data[(msg->dlc - 1) - i];
      }
      can_data |= can_sep_data[i] << i*8;
    }

    tmp_unsigned_data = (can_data >> (msg->dlc * 8 - start_bit - length)) & data_mask;
    RCLCPP_INFO(rclcpp::get_logger(node_name), "CAN DATA %04llx",can_data);

    if(value_type == "Signed")
    {
      if(tmp_unsigned_data >> length - 1 == 0 ){
        velocity = tmp_unsigned_data * factor + offset; //velocity = km/h
      }
      else{
        tmp_signed_data = tmp_unsigned_data | (~0 << length);
        velocity = tmp_signed_data * factor + offset; //velocity = km/h
      }
    }
    else if(value_type == "Unsigned")
    {
      velocity = tmp_unsigned_data * factor + offset; //velocity = km/h
    }

    msg_velocity.twist.linear.x = velocity / 3.6;

    RCLCPP_INFO(rclcpp::get_logger(node_name), "RAW CAN DATA %02x%02x%02x%02x%02x%02x%02x%02x",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
    RCLCPP_INFO(rclcpp::get_logger(node_name), "DATA %04llx",tmp_unsigned_data);
    RCLCPP_INFO(rclcpp::get_logger(node_name), "%lf m/s", msg_velocity.twist.linear.x);
    pub->publish(msg_velocity);

    }
}

int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("can_velocity_converter");

  node->declare_parameter("can_id",can_id);
  node->declare_parameter("start_bit",start_bit);
  node->declare_parameter("length",length);
  node->declare_parameter("factor",factor);
  node->declare_parameter("offset",offset);
  node->declare_parameter("byte_order",byte_order);
  node->declare_parameter("value_type",value_type);

  node->get_parameter("can_id",can_id);
  node->get_parameter("start_bit",start_bit);
  node->get_parameter("length",length);
  node->get_parameter("factor",factor);
  node->get_parameter("offset",offset);
  node->get_parameter("byte_order",byte_order);
  node->get_parameter("value_type",value_type);

  std::cout<< "can_id "<<can_id<<std::endl;
  std::cout<< "start_bit "<<start_bit<<std::endl;
  std::cout<< "length "<<length<<std::endl;
  std::cout<< "factor "<<factor<<std::endl;
  std::cout<< "offset "<<offset<<std::endl;
  std::cout<< "byte_order "<<byte_order<<std::endl;
  std::cout<< "value_type "<<value_type<<std::endl;


  auto sub1 = node->create_subscription<can_msgs::msg::Frame>("/vehicle/can_tx", rclcpp::QoS(10), can_callback);
  pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/can_twist", rclcpp::QoS(10));

  rclcpp::spin(node);

  return 0;
}
