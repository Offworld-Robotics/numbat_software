// WORKING MANUAL INPUT CODE IN drive_arm_manual_input

#include <Servo.h>
#define BAUD_RATE 9600
#define PIN_TOP 13
#define PIN_BOTTOM 12

//1999 is out
//1100 is in
//max min values for actuators
#define MAX 1000
#define MIN 2000
// SERVO VALUES DEALS WITH POSITION NOT VELOCITY
#define FULL_EXTENSION 1750
#define FULL_RETRACTION 1250
//int servopin = 4; 
//int pulse = 1500; 
Servo top, bottom;

void setup() {
  //pinMode(servopin, OUTPUT);
  //Serial.begin(BUAD_RATE);
  top.attach(PIN_TOP, MIN, MAX);
  //bottom.attach(PIN_BOTTOM, MIN, MAX);
  
  //default position
  top.writeMicroseconds(2000);
  //bottom.writeMicroseconds(1500);
  //bottom.writeMicroseconds(2000);
  //Serial.begin(9600);
}

void loop() {
  //digitalWrite(servopin, HIGH);
  //delayMicroseconds(pulse);
  //digitalWrite(servopin, LOW);
  //delay(20);
  
  /*float topNum = Serial.parseFloat();
  float voidNum = Serial.parseFloat();
  Serial.println(Serial.parseFloat());
  float bottomNum = Serial.parseFloat();
  voidNum = Serial.parseFloat();
  Serial.println(Serial.parseFloat());

  if(topNum > MAX){
    topNum = MAX;
  }
  if(topNum < MIN){
    topNum = MIN;
  }
  if(bottomNum > MAX){
    bottomNum = MAX;
  }
  if(bottomNum < MIN){
    bottomNum = MIN;
  }
  

  Serial.println(topNum);
  Serial.println(bottomNum);
  //topDrive = Serial.parseFloat();  
  //bottomDrive = Serial.parseFloat();
  
  // higher value = more extension
  */
  top.writeMicroseconds(1000);
  //top.writeMicroseconds(topNum);

  //top.writeMicroseconds(topDrive);
  //bottom.writeMicroseconds(bottomDrive);
  //delay(10);
  
  //bottom.writeMicroseconds(bottomDrive);
  
  
}

