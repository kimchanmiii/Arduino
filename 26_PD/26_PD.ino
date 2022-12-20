#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_VAR A3

// Event interval parameters
#define _INTERVAL_DIST    20 // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20 // servo interval (unit: ms) 20
#define _INTERVAL_SERIAL  80 // serial interval (unit: ms) 80

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.7      // EMA weight of new sample (range: 0 to 1)

// Servo adjustment
#define _DUTY_NEU 1000   // Servo angle: 0 degree
#define _DUTY_MAX 1500  // Servo angle: D degree // down 1350
#define _DUTY_MIN 300   // Servo angle: E degree // up 820
#define _SERVO_ANGLE_DIFF 160 // Replace with |D - E| degree
#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)

// PID parameters
#define _KP 2.2 // proportional gain 1.8
#define _KD 140 // derivative gain 6.0
//#define _KI 0 // integral gain

// global variables
float dist_filtered, dist_ema, dist_target; // unit: mm

Servo myservo;

int duty_change_per_interval; // maximum duty difference per interval
int duty_target;    // Target duty
int duty_curr;      // Current duty

unsigned long last_sampling_time_dist;   // unit: msec
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec

bool event_dist, event_servo, event_serial; // event triggered?

float error_curr, error_prev, control, pterm, dterm, iterm; // PID control variables

void setup()
{
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);    

  // convert angular speed into duty change per interval.
  duty_change_per_interval = 
    (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / (float) _SERVO_ANGLE_DIFF) * (_INTERVAL_SERVO / 1000.0);
  
  // initialize serial port
  Serial.begin(1000000);
  
  // Set a target distance
  dist_target = 155; // the center of the rail (unit: mm)  
}
  
void loop()
{
  unsigned long time_curr = millis();
  
  // wait until next event time
  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }

  if(event_dist) {
    event_dist = false;
    
    // Get a distance reading from the distance sensor
    dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5));
    dist_ema = _EMA_ALPHA * dist_ema + (1.0 - _EMA_ALPHA) * dist_filtered;

    // Update PID control variables
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm;
    duty_target = _DUTY_NEU + control;
    error_prev = error_curr;

    if(dist_ema >= 128 && dist_ema <= 182) digitalWrite(PIN_LED, 0);
    else digitalWrite(PIN_LED, 1);
  }
  
  if(event_servo) {
    event_servo = false;
     
    // adjust duty_curr toward duty_target by duty_change_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_change_per_interval;
      if (duty_curr > duty_target)
          duty_curr = duty_target;
    } else {
      duty_curr -= duty_change_per_interval;
      if (duty_curr < duty_target)
        duty_curr = duty_target;
    }
    // update servo position
    if(duty_curr > _DUTY_MAX) duty_curr = _DUTY_MAX; // for servo arm protection
    if(duty_curr < _DUTY_MIN) duty_curr = _DUTY_MIN;
    myservo.writeMicroseconds(duty_curr);
  }

///////// DO NOT MODIFY SERIAL OUTPUT!! ///////// 
  if(event_serial) { 
    event_serial = false;

    // use for debugging
    if(0) {
      Serial.print(",ERROR:"); Serial.print(error_curr); 
      Serial.print(",pterm:"); Serial.print(pterm);
      Serial.print(",dterm:"); Serial.print(dterm);
      Serial.print(",duty_target:"); Serial.print(duty_target);
      Serial.print(",duty_curr:"); Serial.print(duty_curr);
    }
    Serial.print("duty_curr ::: "); Serial.print(duty_curr);
    Serial.print("MIN:0,MAX:310,TARGET:155,TG_LO:128,TG_HI:182,DIST:"); 
    Serial.println(dist_ema);
  }
}
/////////////////////////////////////////////////

float volt_to_distance(int x)
{
  // Replace this with the equation obtained from nonlinear regression analysis
  // return (6762.0 / (a_value - 9) - 4.0) * 7.5; 
  // return (521 + -1.62*x + 1.27*pow(10,-3)*x*x);
  return (528 + -1.57*x + 1.2*pow(10,-3)*x*x);
}

unsigned int ir_sensor_filtered(unsigned int n, float position)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  // n: number of measurement repetition
  // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)

  // The output of Sharp infrared sensor includes lots of spiky noise.
  // To eliminate such a spike, ir_sensor_filtered() performs the following two steps:
  // Step 1. Repeat measurement n times and collect n * position smallest samples, where 0 <= postion <= 1.
  // Step 2. Return the largest sample among the collected ones.
  
  unsigned int *ir_val, tmp, ret_idx, ret_val;
  unsigned int start_time;
 
  ret_idx = (unsigned int) ceil(n * position);
  
  // Step 1. Repeat measurement n times and collect n * position smallest samples.
  // Note: simple implementation requires an array of n elements to store n samples.
  // Instead, we can save memory by allocating an array of (n * position + 1) elements.

  ir_val = (unsigned int*) malloc(sizeof(unsigned int) * (ret_idx + 2));
  ir_val[0] = analogRead(PIN_IR);
  
  for (int i = 1; i < n; i++) {
    int j;
    if(i < ret_idx + 1) {
      ir_val[i] = analogRead(PIN_IR);
      j = i - 1;
    }
    else {
      ir_val[ret_idx + 1] = analogRead(PIN_IR);
      j = ret_idx;
    }

    for( ; j >= 0; j--) {
      if(ir_val[j] > ir_val[j+1]) {
        tmp = ir_val[j];
        ir_val[j] = ir_val[j+1];
        ir_val[j+1] = tmp;
      }
    }
  }

  // Step 2. Return the largest sample among the collected ones.
  if(position > 0.0) ret_val = ir_val[ret_idx];
  else ret_val = ir_val[0];
     
  free(ir_val);

  return ret_val;
}
