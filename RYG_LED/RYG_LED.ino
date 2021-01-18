// 매크로 상수 선언------------------------------
#define RLED_PIN    8
#define YLED_PIN    9
#define GLED_PIN    10


// 전역변수 선언---------------------------------


// 초기화 함수-----------------------------------
void setup() {
  // 센서 초기화
  pinMode(RLED_PIN, OUTPUT);
  pinMode(YLED_PIN, OUTPUT);
  pinMode(GLED_PIN, OUTPUT);
}

// 기능 구현 함수--------------------------------
void loop() {
  controlLED(HIGH, LOW, LOW);
  delay(500);

  controlLED(HIGH, HIGH, LOW);
  delay(500);

  controlLED(HIGH, HIGH, HIGH);
  delay(500);

  controlLED(LOW, LOW, LOW);
  delay(500);
  
}

// 3개 LED 제어 함수 ---------------------------
void controlLED(int r, int y, int g){
  digitalWrite(RLED_PIN, r);
  digitalWrite(YLED_PIN, y);
  digitalWrite(GLED_PIN, g);
}
