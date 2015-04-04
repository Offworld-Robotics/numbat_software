#include <Servo.h>
#define BAUD_RATE 9600
#define PIN_TOP 13
#define PIN_BOTTOM 12


//1999 is out
//1100 is in
//max min values for actuators
//#define MAX 1050
//#define MIN 2000
#define MAX 2000
#define MIN 1100
//int servopin = 4; 
//int pulse = 1500; 
Servo top, bottom;

void setup() {
  //pinMode(servopin, OUTPUT);
  //Serial.begin(BUAD_RATE);
  top.attach(PIN_TOP, MIN, MAX);
  //bottom.attach(PIN_BOTTOM, MIN, MAX);
  
  //default position
  //top.writeMicroseconds(1500);
  //bottom.writeMicroseconds(1500);
  //bottom.writeMicroseconds(2000);
}

void loop() {
  //digitalWrite(servopin, HIGH);
  //delayMicroseconds(pulse);
  //digitalWrite(servopin, LOW);
  //delay(20);
  
  float topDrive  = 1500;
  float bottomDrive = 1500;
  
  //topDrive = Serial.parseFloat();  
  //bottomDrive = Serial.parseFloat();
  
  // higher value = more extension
  //bottom.writeMicroseconds(1500);
  top.writeMicroseconds(1500);
  

  //top.writeMicroseconds(topDrive);
  //bottom.writeMicroseconds(bottomDrive);
  //delay(10);
  
  //bottom.writeMicroseconds(bottomDrive);
  
  
}
