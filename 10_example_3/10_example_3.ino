#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  myservo.write(0);
  delay(1000);

  Serial.begin(57600);
}

void loop() {
  int time1, s;
  time1 = millis();
  myservo.write(180);
  s = (millis() - time1)/180;
  Serial.println(s);
  delay(1000);
}
