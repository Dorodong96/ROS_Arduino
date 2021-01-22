/* 
 * ROS Serial Subscriber Code
 * NODE OBJECT  : sub_motor
 * TOPIC NAME   : motor_msg
 * MESSAGE TYPE : std_msgs/String
 * DESCRIPTION  : 명령에 따른 모터 제어(Forward, BackWard, Stop)
 */
/* 
 * ROS Serial Publisher Code
 * NODE OBJECT  : pub_sonic
 * TOPIC NAME   : ultrasound_msg
 * MESSAGE TYPE : sensor_msgs/Range
 * DESCRIPTION  : 측정된 거리 값 전송
 */
 
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

// 매크로 상수 선언
#define LENA   11 // Left Motor
#define LIN1   10   
#define LIN2   9   
#define RIN1   8  // Right Motor
#define RIN2   7
#define RENA   6

#define RLED_PIN      2
#define YLED_PIN      12
#define GLED_PIN      13

#define TRIG_PIN      3
#define ECHO_PIN      4
#define INTERVAL      500  // 0.5sec

#define P_TOPIC_NAME  "ultrasound_msg"
#define S_TOPIC_NAME  "motor_msg"
#define DEBUG         false

ros::NodeHandle nh;

sensor_msgs::Range  range_msg;
ros::Publisher  pub_sonic(P_TOPIC_NAME, &range_msg);

void messageCb(const std_msgs::String& motor_msg) {
  String MTR = motor_msg.data;
  if (DEBUG) nh.loginfo(MTR.c_str());
  
  if (MTR == "forward"){
    if (DEBUG) nh.loginfo("FORWARD");
    controlLED(LOW, LOW, HIGH);
    forward();
  } else if (MTR == "backward"){
    if (DEBUG) nh.loginfo("BACKWARD");
    controlLED(LOW, HIGH, LOW);
    backward();
  } else if (MTR == "stop"){
    if (DEBUG) nh.loginfo("STOP");
    controlLED(HIGH, LOW, LOW);
    stop_pos();
  } else if (MTR == "right"){
    if (DEBUG) nh.loginfo("RIGHT");
    right();
  } else if (MTR == "left"){
    if (DEBUG) nh.loginfo("LEFT");
    left();
  }
}

ros::Subscriber<std_msgs::String> sub_motor(S_TOPIC_NAME, &messageCb);

// 시간계산 변수
unsigned long range_time = 0;

void setup() {
  // 센서 초기화 (Motor)
  // Motor Connect PIN
  pinMode(LENA, OUTPUT); 
  pinMode(LIN1, OUTPUT); 
  pinMode(LIN2, OUTPUT);
  pinMode(RIN1, OUTPUT); 
  pinMode(RIN2, OUTPUT);
  pinMode(RENA, OUTPUT); 

  // Motor Speed Init 0(0V)~255(5V)
  analogWrite(LENA, 150);
  analogWrite(RENA, 150);
  
  // 센서 초기화 (Ultrasound)
  pinMode(TRIG_PIN, OUTPUT);  //Arduino --> Ultrasound
  pinMode(ECHO_PIN, INPUT);   //Ultrasound --> Arduino

  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(ECHO_PIN, LOW);

  // 센서 초기화 (LED)
  pinMode(RLED_PIN, OUTPUT);
  pinMode(YLED_PIN, OUTPUT);
  pinMode(GLED_PIN, OUTPUT);

  // 노드 초기화
  nh.initNode();
  nh.subscribe(sub_motor);
  nh.advertise(pub_sonic);

  // MESSAGE 초기화
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = P_TOPIC_NAME;
  range_msg.field_of_view = 0.1;
  range_msg.min_range = 0.02; // 2cm --> 0.02m
  range_msg.max_range = 4;
}

void loop() {
  if (millis() - range_time >= INTERVAL) {
    // MESSAGE 생성
    range_msg.range=getDistance();
    range_msg.header.stamp=nh.now();
    
    // MESSAGE 발송
    pub_sonic.publish(&range_msg);

    // DEBUG (loginfo가 pointer변수를 받으므로 c_str()사용)
    if(DEBUG) nh.loginfo(String(range_msg.range).c_str());
    
    range_time = millis();
    nh.spinOnce();
  }
  
  nh.spinOnce();
  delay(1);
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

void right() {
  digitalWrite(LIN1, LOW);
  digitalWrite(LIN2, LOW);
  digitalWrite(RIN1, HIGH);
  digitalWrite(RIN2, LOW);
}

void left() {
  digitalWrite(LIN1, HIGH);
  digitalWrite(LIN2, LOW);
  digitalWrite(RIN1, LOW);
  digitalWrite(RIN2, LOW);
}

// 3개 LED 제어 함수 ---------------------------
void controlLED(int r, int y, int g){
  digitalWrite(RLED_PIN, r);
  digitalWrite(YLED_PIN, y);
  digitalWrite(GLED_PIN, g);
}
