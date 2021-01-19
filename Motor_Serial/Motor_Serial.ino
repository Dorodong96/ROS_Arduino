/* 
 * ROS Serial Subscriber Code
 * NODE OBJECT  : sub_motor
 * TOPIC NAME   : motor_msg
 * MESSAGE TYPE : std_msgs/String
 * DESCRIPTION  : 명령에 따른 모터 제어(Forward, BackWard, Stop)
 */
#include <ros.h>
#include <std_msgs/String.h>

#define LIN1        6   // Left Motor
#define LIN2        9   
#define RIN1        10  // Right Motor
#define RIN2        11
#define TOPIC_NAME  "motor_msg"
#define DEBUG       false

void messageCb(const std_msgs::String& motor_msg) {
  String MTR = motor_msg.data;
  
  if (DEBUG)  nh.loginfo(
  if (MTR == "forward")
    forward();
  else if (MTR == "backward")
    backward();
  else if (MTR == "stop")
    stop_pos();
}

// 전역변수 선언
ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub_motor(TOPIC_NAME, &messageCb);

// 초기화 함수
void setup() {
  // 센서 초기화
  pinMode(LIN1, OUTPUT); 
  pinMode(LIN2, OUTPUT);
  pinMode(RIN1, OUTPUT); 
  pinMode(RIN2, OUTPUT);

  // 노드 초기화
  nh.initNode();
  nh.subscribe(sub_motor);
}

// 기능 구현 함수
void loop() {
  nh.spinOnce();
  delay(1);
}

// 모터제어 함수
void forward() {
  digitalWrite(LIN1, HIGH);
  digitalWrite(LIN2, LOW);
  digitalWrite(RIN1, HIGH);
  digitalWrite(RIN2, LOW);
}

void backward() {
  digitalWrite(LIN1, LOW);
  digitalWrite(LIN2, HIGH);
  digitalWrite(RIN1, LOW);
  digitalWrite(RIN2, HIGH);
}

void stop_pos() {
  digitalWrite(LIN1, LOW);
  digitalWrite(LIN2, LOW);
  digitalWrite(RIN1, LOW);
  digitalWrite(RIN2, LOW);
}
