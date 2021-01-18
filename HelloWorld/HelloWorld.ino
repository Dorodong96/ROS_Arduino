/*
 * rosserial 퍼블리셔 코드
 * Node Object  : chater
 * Topic Name   : "chatter"
 * Message Type : std_msgs/String
 */

#include <ros.h>
#include <std_msgs/String.h>

// 전역변수 선언
// ROS 관련
ros::NodeHandle  nh;
std_msgs::String str_msg;  // std_msgs/String(객체)  str_msgs(변수 선언)
ros::Publisher chatter("chatter", &str_msg);

// Data 관련
char hello[13] = "hello world!";

void setup()
{
  // ROS에게 Publisher 정보 전달
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  
  chatter.publish( &str_msg );   // 문자열을 주소값으로 전달
  
  nh.spinOnce();
  
  delay(1000);
}
