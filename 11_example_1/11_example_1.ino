#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servi motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.3    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// Target Distance
#define _TARGET_LOW  180.0
#define _TARGET_HIGH 360.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
 
#define _DUTY_MIN 550 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1575 // servo neutral position (90 degree)
#define _DUTY_MAX 2600 // servo full counterclockwise position (180 degree)

// global variables
float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time; // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float  dist_raw;
  
  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // cut lower than minimum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON      
    dist_prev = dist_raw;
  }

  // Apply ema filter here  
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // adjust servo position according to the USS read value

  // add your code here!
  if (dist_ema <= _TARGET_LOW){
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  else if (dist_ema > _TARGET_LOW && dist_ema < _TARGET_HIGH ){
    myservo.writeMicroseconds((dist_ema - _DIST_MIN) / (_TARGET_HIGH - _TARGET_LOW) * (_DUTY_MAX - _DUTY_MIN) + _DUTY_MIN);    
  }
  else if (dist_ema >= _TARGET_HIGH){
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  // Use _TARGET_LOW, _TARGTE_HIGH

  // output the distance to the serial port
  //Serial.print(",Low:");   Serial.print(_TARGET_LOW);
  //Serial.print(",High:");  Serial.print(_TARGET_HIGH);
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - round trip distance: 34.6m
  // - expected pulse duration: 0.1 sec, or 100,000us
  // - pulseIn(ECHO, HIGH, timeout) * 0.001 * 0.5 * SND_VEL
  //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //        = 100,000 * 0.001 * 0.5 * 346 * micro * sec * milli * meter
  //                                        ----------------------------
  //                                         micro * sec
  //        = 100 * 173 milli*meter = 17,300 mm = 17.3m
  // pulseIn() returns microseconds.
}
