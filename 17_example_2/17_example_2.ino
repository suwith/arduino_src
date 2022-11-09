#include <Servo.h>

// Arduino pin assignment
#define PIN_POTENTIOMETER 3 // Potentiometer at Pin A3
// Add IR Sensor Definition Here !!!
#define PIN_SERVO 10
#define PIN_LED 9

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)

#define LOOP_INTERVAL 50 // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time; // unit: msec
int analogPin = 0;
float dist;
float dist_ema;
float alpha = 0.5;
void setup()
{
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(57600);
}

void loop()
{
  unsigned long time_curr = millis();
  int a_value, duty;
  a_value = analogRead(analogPin);
  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  dist = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;
  dist_ema = alpha * dist + (1 - alpha) * dist_ema;
  if (dist_ema <= 100.0)
  {
    myservo.writeMicroseconds(_DUTY_MIN);
    analogWrite(PIN_LED, 255);
  }
  else if (dist_ema > 100.0 && dist_ema < 250.0)
  {
    double i = (dist_ema - 180) * 10.25;
    duty = i + 553;
    myservo.writeMicroseconds(duty);
    analogWrite(PIN_LED, 0);
  }
  else
  {
    myservo.writeMicroseconds(_DUTY_MAX);
    analogWrite(PIN_LED, 255);
  }
  // Read IR Sensor value !!!
  // Convert IR sensor value into distance !!!
  // we need distance range filter here !!!
  // we need EMA filter here !!!

  // map distance into duty

  // print IR sensor value, distnace, duty !!!
  Serial.print(",IR:");
  Serial.print(a_value);
  Serial.print(",dist:");
  Serial.print(dist);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(duty);
  Serial.println("");
}
