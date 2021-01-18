/* 
 * ROS Serial Subscriber Code
 * Node Object  : sub_led
 * Topic Name   : "ryg_led"
 * Mesage Type  : std_msgs/String
 */

#include <ros.h>
#include <std_msgs/String.h>

#define RLED_PIN    8
#define YLED_PIN    9
#define GLED_PIN    10
#define TOPIC_NAME  "ryg_led"
#define DEBUG       false

// 전역변수 선언---------------------------------
ros::NodeHandle nh;

// Callback Function (메시지를 수신했을 때 실행되는 함수)
void messageCb(const std_msgs::String& led_msg){
  
  String LED = led_msg.data;
  if(DEBUG) nh.loginfo("Error");
  
  if (LED == "r"){
      if(DEBUG) nh.loginfo("R OK");
      controlLED(HIGH - digitalRead(RLED_PIN), LOW, LOW);
  } else if (LED == "y") {
      controlLED(LOW, HIGH - digitalRead(YLED_PIN), LOW);
  } else if (LED == "g") {
      controlLED(LOW, LOW, HIGH - digitalRead(GLED_PIN));
  }
}

ros::Subscriber<std_msgs::String> sub_led(TOPIC_NAME, &messageCb);

// 초기화 함수-----------------------------------
void setup() {
  // 센서 초기화
  pinMode(RLED_PIN, OUTPUT);
  pinMode(YLED_PIN, OUTPUT);
  pinMode(GLED_PIN, OUTPUT);

  // 노드 초기화
  nh.initNode();
  nh.subscribe(sub_led);
}

// 기능 구현 함수--------------------------------
void loop() {
  nh.spinOnce();
  delay(1);
}

// 3개 LED 제어 함수 ---------------------------
void controlLED(int r, int y, int g){
  digitalWrite(RLED_PIN, r);
  digitalWrite(YLED_PIN, y);
  digitalWrite(GLED_PIN, g);
}
