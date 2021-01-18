// PIN Define
#include "header.h"
#define LED_PIN   13

// Init ----------------------- just for once
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);   // Data Out to Pin 13
}

// Continue ---------------------
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_PIN, HIGH);  // LED ON
  delay(1000);                  // 1sec
  digitalWrite(LED_PIN, LOW);   // LED OFF
  delay(1000);                  // 1sec
}
