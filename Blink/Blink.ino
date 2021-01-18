/* 
 * rosserial Subscriber
 * 
 * Node Object : 
 * Topic Name : toggle_led
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

// Callback Function (메시지를 수신했을 때 실행되는 함수)
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

// Topic : toggle_led가 발생했을 때 처리할 messageCb 함수
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb ); 

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
