/* 
 * ROS Serial Publisher Code
 * NODE OBJECT  : pub_sonic
 * TOPIC NAME   : ultrasound_msg
 * MESSAGE TYPE : sensor_msgs/Range
 * DESCRIPTION  : 측정된 거리 값 전송
 */
 
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define TRIG_PIN    3
#define ECHO_PIN    4
#define INTERVAL    1000  // 0.1sec

#define TOPIC_NAME  "ultrasound_msg"

// ROS 전역변수
ros::NodeHandle nh;
sensor_msgs::Range  range_msg;
ros::Publisher  pub_sonic(TOPIC_NAME, &range_msg);

// 시간계산 변수
unsigned long range_time = 0;

void setup() {
  // 센서 초기화
  pinMode(TRIG_PIN, OUTPUT);  //Arduino --> Ultrasonic
  pinMode(ECHO_PIN, INPUT);   //Ultrasonic --> Arduino

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(ECHO_PIN, LOW);
  
  // ROS 초기화
  nh.initNode();
  nh.advertise(pub_sonic);

  // MESSAGE 초기화
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = TOPIC_NAME;
  range_msg.field_of_view = 0.1;
  range_msg.min_range = 0.02; // 2cm --> 0.02m
  range_msg.max_range = 4;
 }

// 기능 구현 함수
void loop() {

  if (millis() - range_time >= INTERVAL) {
    // MESSAGE 생성
    range_msg.range=getDistance();
    range_msg.header.stamp=nh.now();
    
    // MESSAGE 발송
    pub_sonic.publish(&range_msg);

    // DEBUG
    nh.loginfo(String(range_msg.range).c_str());
    range_time = millis();

    nh.spinOnce();
  }

}

// Ultrasonic 제어 함수

float getDistance(){
  // (1)발사명령 신호 (Pulse)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // 이 상태는 유지해야함(HIGH 10ms)
  digitalWrite(TRIG_PIN, LOW);

  //(2) 반사신호 시간(microseconds) & 거리 계산
  unsigned long duration = pulseIn(ECHO_PIN, HIGH); // pulseIn : 들어온 신호 시간(msec)
  float distance = ((float)(340 * duration)/1000000)/2; // 340m/sec

  //(3) 거리값 반환
  return distance; // 2cm ~ 4m 측정 가능
}
