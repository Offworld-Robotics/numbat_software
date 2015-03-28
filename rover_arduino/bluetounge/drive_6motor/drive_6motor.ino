#include <Servo.h>
#define BAUD_RATE 9600


#define LF_PIN 8
#define LM_PIN 9
#define LB_PIN 10
#define RF_PIN 11
#define RM_PIN 12
#define RB_PIN 13


#define FULL_FORWARD  2000.0
#define FULL_BACKWARD 1000.0
#define RANGE (FULL_FORWARD - FULL_BACKWARD)

#define MAX_IN 0.75
#define MIN_IN -0.75

Servo leftFront, leftMiddle, leftBack;
Servo rightFront, rightMiddle, rightBack;


void setup() {
  //open serial port
  Serial.begin(BAUD_RATE);
  leftFront.attach(LF_PIN);
  leftMiddle.attach(LM_PIN);
  leftBack.attach(LB_PIN);
  rightFront.attach(RF_PIN);
  rightMiddle.attach(RM_PIN);
  rightBack.attach(RB_PIN);
  
  leftFront.writeMicroseconds(1500);
  leftMiddle.writeMicroseconds(1500);
  leftBack.writeMicroseconds(1500);
  rightFront.writeMicroseconds(1500);
  rightMiddle.writeMicroseconds(1500);
  rightBack.writeMicroseconds(1500);
}

/*void setSide(float drive, Servo front, Servo middle, Servo back) {
  //prevent going over max
  /*if(drive > MAX_IN) {
    drive = MAX_IN;  
  } else if (drive < MIN_IN) {
    drive = MIN_IN;
  }*/

  //int microsecs = (int)(((float)(drive / (float)MAX_IN)*(float)(RANGE)) + (float)1000);
/*  int microsecs = (int)drive;
  //Serial.println(microsecs);
  front.writeMicroseconds(microsecs);
  middle.writeMicroseconds(microsecs);
  back.writeMicroseconds(microsecs);
  
}*/

void setMotorSpeed(float drive, Servo motor){
  
  //int microsecs = (int) drive;
  //motor.writeMicroseconds(microsecs);
  motor.writeMicroseconds((int) drive);
  
}

void loop() {
  float leftFrontDrive  = 0.0;
  float leftMiddleDrive  = 0.0;
  float leftBackDrive  = 0.0;

  float rightFrontDrive = 0.0;
  float rightMiddleDrive = 0.0;
  float rightBackDrive = 0.0;

      leftFrontDrive = Serial.parseFloat();  
      leftMiddleDrive = Serial.parseFloat();
      leftBackDrive = Serial.parseFloat();  
      rightFrontDrive = Serial.parseFloat();
      rightMiddleDrive = Serial.parseFloat();  
      rightBackDrive = Serial.parseFloat();
      //Serial.println(leftDrive);
      //Serial.println(rightDrive);
//      setSide(leftDrive, leftFront, leftMiddle,leftBack);
//      setSide(rightDrive, rightFront, rightMiddle,rightBack);
      //Servo.refresh();
      
      setMotorSpeed(leftFrontDrive, leftFront);
      setMotorSpeed(leftMiddleDrive, leftMiddle);
      setMotorSpeed(leftBackDrive, leftBack);
      setMotorSpeed(rightFrontDrive, rightFront);
      setMotorSpeed(rightMiddleDrive, rightMiddle);
      setMotorSpeed(rightBackDrive, rightBack);
  
}
