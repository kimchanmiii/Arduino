#include <Servo.h>

// Arduino pin assignment

// #define PIN_POTENTIOMETER 3 // Potentiometer at Pin A3
// Add IR Sensor Definition Here !!!
#define PIN_IR_SENSOR 0
#define PIN_SERVO 10
#define PIN_LED 9

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)

#define EMA 0.5

#define LOOP_INTERVAL 50   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec
int ema;

void setup()
{
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(2000000);
}

long _map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{
  unsigned long time_curr = millis();
  int a_value, duty, dist;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  // Remove Next line !!!
  // a_value = analogRead(PIN_POTENTIOMETER);
  // Read IR Sensor value !!!
  a_value = analogRead(PIN_IR_SENSOR);
  // Convert IR sensor value into distance !!!
  dist = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;
  // we need distance range filter here !!!
  if (dist < 50.0 || dist > 300.0) return;
  if (dist >= 100.0 && dist <= 250.0) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
    if(dist < 100.0) {
      dist = 100.0;
    } else {
      dist = 250.0;
    }
  }

  // we need EMA filter here !!!
  ema = ema * EMA + dist * (1 - EMA);

  // map distance into duty
  // duty = map(a_value, 0, 1023, _DUTY_MIN, _DUTY_MAX);
  duty = _map(ema, 100, 250, _DUTY_MIN, _DUTY_MAX);
  // duty = (ema - 100.0 / 150.0) * (_DUTY_MAX - _DUTY_MIN) + _DUTY_MIN;
  myservo.writeMicroseconds(duty);

  // print IR sensor value, distnace, duty !!!
  Serial.print("ADC Read: "); Serial.print(a_value);
  Serial.print(" ,Duty: ");
  Serial.print(duty);
  Serial.print(" ,dist: ");
  Serial.print(dist);
  Serial.print(" ,ema: ");
  Serial.println(ema);
}
