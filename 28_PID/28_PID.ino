#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_VAR A3

// Event interval parameters
//////////////// DO NOT modify below section!! /////////////////////////        
#define _INTERVAL_DIST    20 // distance sensor interval (unit: ms)
#define _INTERVAL_SERVO   20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL  80 // serial interval (unit: ms)
#define _INTERVAL_MOVE    10000 // mission interval (unit: ms)
////////////////////////////////////////////////////////////////////////

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.8   // EMA weight of new sample (range: 0 to 1)

// Servo adjustment
#define _DUTY_NEU 1000 // Servo angle: 0 degree
#define _DUTY_MAX 1500  // Servo angle: D degree
#define _DUTY_MIN 300   // Servo angle: E degree 500
#define _SERVO_ANGLE_DIFF 60 // Replace with |D - E| degree
#define _SERVO_SPEED 100 // servo speed limit (unit: degree/second)

// PID parameters
#define _DIST_TARGET 155 // center of the rail (unit: mm)
#define _KP 5.2 // proportional gain
#define _KD 400  // derivative gain
#define _KI 0.015  // integral gain
#define _ITERM_MAX 150 // uncomment if necessary

//////////////// DO NOT modify below section!! /////////////////////////        
unsigned long error_sum, error_cnt, toggle_cnt;
////////////////////////////////////////////////////////////////////////

// global variables
float dist_filtered, dist_ema; // unit: mm
Servo myservo;

int duty_change_per_interval; // maximum duty difference per interval
int duty_target;    // Target duty
int duty_curr;      // Current duty
int dist_target; 
unsigned long last_sampling_time_dist;   // unit: msec
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec
unsigned long last_sampling_time_move; // unit: msec
bool event_dist, event_servo, event_serial; // event triggered?
float error_curr, error_prev, control, pterm, dterm, iterm;

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
//////////////// DO NOT modify below section!! /////////////////////////        
  dist_target = 55;
////////////////////////////////////////////////////////////////////////

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

//////////////// DO NOT modify below section!! /////////////////////////        
  if (time_curr >= (last_sampling_time_move + _INTERVAL_MOVE)) {
        last_sampling_time_move += _INTERVAL_MOVE;
        dist_target = 310 - dist_target;
        if(++toggle_cnt == 5) {
          Serial.println("----------------------------------------------------"),
          Serial.print("ERR_cnt:"),
          Serial.print(error_cnt);
          Serial.print(",ERR_sum:"),
          Serial.print(error_sum);
          Serial.print(",ERR_ave:"),
          Serial.println(error_sum / (float) error_cnt);
          exit(0);
        }
  }
////////////////////////////////////////////////////////////////////////

  if(event_dist) {
    event_dist = false;
    
    // Get a distance reading from the distance sensor
    dist_filtered = volt_to_distance(ir_sensor_filtered(10, 0.5));
    dist_ema = _EMA_ALPHA * dist_ema + (1.0 - _EMA_ALPHA) * dist_filtered;

    // Update PID variables
    error_curr = dist_target - dist_ema;

//////////////// DO NOT modify below section!! /////////////////////////
    if(time_curr >= last_sampling_time_move + 3000) {
      error_cnt++;
      if(abs(error_curr) > 3) {
        error_sum += abs(error_curr);
      }
    }
    if(abs(error_curr) <= 3) {
      digitalWrite(PIN_LED, 0);      
    }
    else {
      digitalWrite(PIN_LED, 1);
    }
    
////////////////////////////////////////////////////////////////////////

    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI*error_curr;
    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;
    error_prev = error_curr;
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
    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;

    if(0) { // use for debugging with serial plotter
      Serial.print("MIN:0,MAX:310,TARGET:"); Serial.print(dist_target);
      Serial.print(",DIST:"); 
      Serial.print(dist_ema);
      Serial.print(",ERR_ave(10X):"),
      Serial.println(error_sum / (float) error_cnt * 10);
    }

    if(0) { // use for debugging with serial monitor
      Serial.print("MIN:0,MAX:310,TARGET:"); Serial.print(dist_target);
      Serial.print(",DIST:"); 
      Serial.print(dist_ema);
      Serial.print(",pterm:"); 
      Serial.print(pterm);
      Serial.print(",dterm:"); 
      Serial.print(dterm);
      Serial.print(",iterm:"); 
      Serial.print(iterm);
      Serial.print(",ERR_cnt:"),
      Serial.print(error_cnt);
      Serial.print(",ERR_sum:"),
      Serial.print(error_sum);
      Serial.print(",ERR_curr:"),
      Serial.print(abs(error_curr));
      Serial.print(",ERR_ave:"),
      Serial.println(error_sum / (float) error_cnt);
    }

    if(1) { // use for evaluation
      Serial.print("ERR_cnt:"),
      Serial.print(error_cnt);
      Serial.print(",ERR_curr:"),
      Serial.print(abs(error_curr));
      Serial.print(",ERR_sum:"),
      Serial.print(error_sum);
      Serial.print(",ERR_ave:"),
      Serial.println(error_sum / (float) error_cnt);
    }
  }
}

float volt_to_distance(int x)
{
  return (528 + -1.57*x + 1.2*pow(10,-3)*x*x - 30); // Replace this with the equation obtained from nonlinear regression analysis
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
