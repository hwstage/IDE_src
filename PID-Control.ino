#include <Servo.h>
/////////////////////////////
// Configurable parameters //
/////////////////////////////
// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define IR_PIN A0
float dist;
float calcDistance(short val){return dist;}
// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410
// Distance sensor
#define _DIST_ALPHA 0.2
// Servo range
#define _DUTY_MIN 1120
#define _DUTY_NEU 1253
#define _DUTY_MAX 1430
// Servo speed control
#define _SERVO_ANGLE 30.0 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 700 // servo speed limit (deg/sec)
// Event periods
#define _INTERVAL_DIST 20 // distance sensor interval (ms)
#define _INTERVAL_SERVO 20 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)
// PID parameters
#define _KP 1.2
// proportional gain *****
#define _KD 19.5
#define _KI 0.001
#define a 105
#define b 414

//////////////////////
// global variables //
//////////////////////
float interval = 20;
unsigned long oldmil; 
static long apt = 0;
int fc = 15;
float dt = interval/1000.0;
float lambda = 2*PI*fc*dt; 
float filter = 0.0;
float prev = 0.0;
// Servo instance
Servo myservo;
// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;
bool event_dist, event_servo, event_serial;
// Servo speed control
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
int duty_min = _DUTY_MIN;
int duty_neu = _DUTY_NEU;
int duty_max = _DUTY_MAX;
int kp = _KP;
void setup() {
  // initialize GPIO pins for LED add attach servo
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(_DUTY_NEU);
  
  // initialize global variables
  dist_target = _DIST_TARGET;
  dist_raw = 0.0;
//  dist_ema = _DIST_ALPHA*dist_raw + (1 - _DIST_ALPHA)*dist_ema;
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  event_dist = true;
  event_servo = true;
  event_serial = true;

  // initialize serial port
  Serial.begin(57600);
  duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);
  //convert angle speed into duty change per interval.
  duty_chg_per_interval =  (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_DIST / 1000.0);

}
float ir_distance(void){
  float val;
  float volt = float(analogRead(IR_PIN));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (val - a) + 100;
}
float under_noise_filter(void){ 
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < 6; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    delayMicroseconds(1500);
  }
  return largestReading;
}
float ir_distance_filtered(void){ 
  int currReading;
  int lowestReading = 1024;
  dist_raw = ir_distance(); 
  for (int i = 0; i < 6; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }

  dist_ema = _DIST_ALPHA *lowestReading + (1-_DIST_ALPHA )*dist_ema;
  return dist_ema;
}
void loop() {
  unsigned long time_curr = millis();
   if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
     last_sampling_time_dist += _INTERVAL_DIST;
     event_dist = true;
    }
   if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
     last_sampling_time_servo += _INTERVAL_SERVO;
     event_servo = true;
    }
   if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
     last_sampling_time_serial += _INTERVAL_SERIAL;
     event_serial = true;
   }
   
  if(event_dist) {
    event_dist  = false;
    // get a distance reading from the distance sensor
//    dist_raw = ir_distance();
    dist_raw = ir_distance_filtered();
//     dist_raw = filter.read();
  // PID control logic
    error_curr = dist_target - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    if (dterm > -14 && dterm < 14){
      dterm = 0;
    }
    if (pterm > -4 && pterm < 4){
      pterm = 0;
    }
    iterm = _KI * error_curr;
    control = pterm + dterm + iterm;
//    duty_target = targeting(control);
//    duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_MAX - _DUTY_NEU):(_DUTY_NEU - _DUTY_MIN))  * _SERVO_ANGLE / 180;
    duty_target = _DUTY_NEU + control;
    // keep duty_target value within the range of
//     [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;
    }else if (duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    error_prev = error_curr;
  }
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_curr > duty_target){
        if(duty_curr  - duty_chg_per_interval < duty_target){
          duty_curr = duty_target;
        }else{
          duty_curr  -= duty_chg_per_interval;
        }
//    duty_curr = duty_target;
    }else if (duty_curr <= duty_target){
      if (duty_curr + duty_chg_per_interval > duty_target){
        duty_curr == duty_target;
      }else{
        duty_curr  += duty_chg_per_interval;
        }
//    duty_curr = duty_target;
    }
    // update servo position
   myservo.writeMicroseconds(duty_curr);
    }
  if(event_serial) {
    event_serial = false; 
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
//Serial.print(pterm);
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
//    Serial.print(dterm);
    Serial.print(",I:");
//    Serial.print(iterm);
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
  

}

//float ir_distance(void){
//  float val;
//  float volt = float(analogRead(IR_PIN));
//  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
//  return 300.0 / (b - a) * (val - a) + 100;
//}
//float ir_distance_filtered(void){ // return value unit: mm
//   float calidist =ir_distance();
//   unsigned long dmil = 0;
//   unsigned long mil = millis();
//  
//   if (mil != oldmil) {
//     dmil = mil-oldmil; 
//     oldmil = mil;   
//   } 
//  
//   apt -= dmil; 
//  
//   if (apt <= 0) { 
//     apt += interval;  
//     filter = lambda / (1 + lambda) * ir_distance() + 1 / (1 + lambda) * prev; 
//     prev = filter; 
//   return filter;
//
//}
//}
//float under_noise_filter(void){ 
//  int currReading;
//  int largestReading = 0;
//  for (int i = 0; i < 6; i++) {
//    currReading = ir_distance();
//    if (currReading > largestReading) { largestReading = currReading; }
//    delayMicroseconds(1500);
//  }
//  return largestReading;
//}
//float ir_distance_filtered(void){ 
//  int currReading;
//  int lowestReading = 1024;
//  dist_raw = ir_distance(); 
//  for (int i = 0; i < 6; i++) {
//    currReading = under_noise_filter();
//    if (currReading < lowestReading) { lowestReading = currReading; }
//  }
//
//  dist_ema = _DIST_ALPHA *lowestReading + (1-_DIST_ALPHA )*dist_ema;
//  return dist_ema;
//}
float targeting(float e){
  float control;
  if (e >= 0) {
    control = map(e, 0 , 155, duty_neu, duty_max);
  }else if (e < 0){
    control = map(e, -155, 0 , duty_min, duty_neu);
  }
  return control;
}
