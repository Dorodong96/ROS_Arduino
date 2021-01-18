// 라이브러리 포함---------------
#include <ros.h>
#include <std_msgs/String.h>

// 전역변수 선언-----------------
ros::NodeHandle   nh;
std_msgs::String  str_msg;

char hello[] = "HELLO~ROS";
char fatal[] = "fatalities";
int gNum     = 1;

void setup() {
  nh.initNode();
}

void loop() {
  str_msg.data = hello;
  String test = str_msg.data;

  //ROS-------> LOG Printing
  nh.loginfo(String(gNum++).c_str()); // loginfo(char *)
  nh.logwarn(test.c_str());
  nh.logerror("[ ERROR ]");
  nh.logfatal(fatal);

  nh.spinOnce();
  delay(1000);

}
