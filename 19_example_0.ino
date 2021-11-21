// Arduino pin assignment
#include <Servo.h>
#define PIN_LED 9
#define PIN_IR A0
#define PIN_SERVO 10
Servo myservo;
int a, b; // unit: mm

#define _DUTY_MIN 553 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counterclockwise position (180 degree)

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 100; //70;
  b = 400; //300;
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1476);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali < 225){
    myservo.writeMicroseconds(1600);
  }else{
    myservo.writeMicroseconds(1300);
  }
  delay(20);
}
