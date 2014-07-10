 /*
 * Pretends to be a husky. Currently controls left and right drive motors
 * and reads the encoders. 
 * NOTE:
 * - Left side is inverted
 * - The buffers are set to 150 bytes and 5 max of sub and pub, it
 *   doesn't work with the default 280 buffers.
 * TODO:
 * - E-stop signal
 * - Battery voltage read
 * - Deal with timer overflow (happens after 70mins of continous running
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <clearpath_base/Encoders.h>
#include <clearpath_base/Encoder.h>
#include <shadow_base/ShadowStatus.h>
#include <shadow_base/ShadowAuxMove.h>
#include <Servo.h>

// Only have enough interrupt pins for wire A on each encoder
#define ENCODER_RIGHT_INT 0
#define ENCODER_LEFT_INT 1
#define ENC_R_A 2
#define ENC_R_B 4
#define ENC_L_A 3
#define ENC_L_B 5

#define WHEEL_DIAM 0.2
#define ENC_TICKS_PER_REV 360
#define BASE_WIDTH 0.512
#define PI 3.14

#define BAT_VOLT_PIN 0
#define E_STOP_PIN 6

#define DRIVE_CMD_WATCHDOG_MS 800
#define SPEED_CTRL_PERIOD 1
#define ROS_UPDATE_PERIOD 150

Servo leftDrive;
Servo rightDrive;
Servo camPan;
Servo camTilt;
Servo bucket;

int encCountR = 0;
int encCountL = 0;

float leftMotorTarget = 0;
float rightMotorTarget = 0;
float leftMotorSpeed = 0;
float rightMotorSpeed = 0;
float leftMotorPower = 0;
float rightMotorPower = 0;

float printSpeedL = 0;
float printSpeedR = 0;

float maxSpeed = 0.5;
float shadowP = 0.005;

float prevRightDist = 0;
float prevLeftDist = 0;
unsigned long startPeriodL, startPeriodR, endPeriodL, endPeriodR = 0;
unsigned long lastDriveMsgTime = 0;

char *cmd_vel = "/husky/cmd_vel";
char *enc_pub_path = "/husky/data/encoders";
char *status_pub_path = "/shadowStatus";
char *aux_move_path = "/husky/aux_move";

clearpath_base::Encoders encs_msg;
clearpath_base::Encoder encoders[2];
shadow_base::ShadowStatus shadowStatus;
shadow_base::ShadowAuxMove auxMove;

ros::Publisher enc_pub(enc_pub_path, &encs_msg);
ros::Publisher status_pub(status_pub_path, &shadowStatus);
ros::NodeHandle  nh;

void moveBaseCb(const geometry_msgs::Twist& move_msg) {
  float linear = move_msg.linear.x; // In m/s
  float rotate = move_msg.angular.z; // In rads/s
  float innerR = (linear / rotate) - (BASE_WIDTH / 2);
  float scaledLinear = linear / 2;
  //setMotorSpeeds(innerR * rotate, (innerR + BASE_WIDTH) * rotate);
  driveMotors(scaledLinear - rotate/2, scaledLinear + rotate/2);
  //setMotorSpeeds(linear - rotate/4, linear + rotate/4);
  lastDriveMsgTime = millis();
}

void auxMoveCb(const shadow_base::ShadowAuxMove move_msg) {
   driveAux(move_msg.cameraPan.x, move_msg.cameraPan.y, move_msg.bucketSpeed);
}

ros::Subscriber<geometry_msgs::Twist> sub(cmd_vel, &moveBaseCb);
ros::Subscriber<shadow_base::ShadowAuxMove> sub(aux_move_path, &auxMoveCb);

void setup()
{
   leftMotorPower = 0;
   rightMotorPower = 0;
   encs_msg.encoders = encoders;
   encs_msg.encoders_length = 2;
   leftDrive.attach(10);
   rightDrive.attach(9);
   camPan.attach(8);
   camTilt.attach(7);
   bucket.attach(6);

   attachInterrupt(ENCODER_RIGHT_INT, encRInc, RISING);
   attachInterrupt(ENCODER_LEFT_INT, encLInc, RISING);
   pinMode(ENC_R_B, INPUT);
   pinMode(ENC_L_B, INPUT);
   pinMode(E_STOP_PIN, INPUT);
   
   startPeriodL = micros();
   startPeriodR = micros();
   
   nh.initNode();
   nh.subscribe(sub);
   nh.advertise(enc_pub);
   nh.advertise(status_pub);
   driveMotors(0, 0);
   
   //while(!nh.connected()) {
   // nh.spinOnce();
   //}

}

void loop()
{
   delay(SPEED_CTRL_PERIOD);
   static int i = 0;
   i++;
   // Speed maths
   float rightDist = encCountR * ((PI * WHEEL_DIAM) / ENC_TICKS_PER_REV);
   float leftDist = encCountL * ((PI * WHEEL_DIAM) / ENC_TICKS_PER_REV);

   unsigned long timeBetweenTicksR = startPeriodR - endPeriodR;
   unsigned long timeBetweenTicksL = startPeriodL - endPeriodL;
   if (timeBetweenTicksR > 0) {
       rightMotorSpeed = (rightDist - prevRightDist) / ((float)(endPeriodR - startPeriodR) / 1000000);
       startPeriodR = endPeriodR;
       prevRightDist = rightDist;
   }  
   if (timeBetweenTicksL > 0) {
       leftMotorSpeed = ((leftDist - prevLeftDist) / (endPeriodL - startPeriodL)) * 1000000;
       startPeriodL = endPeriodL;
       prevLeftDist = leftDist;
   } 
   printSpeedL += leftMotorSpeed;
   printSpeedR += rightMotorSpeed;
   if (i % ROS_UPDATE_PERIOD/SPEED_CTRL_PERIOD == 0) {
     publishToROS();
     printSpeedL = 0;
     printSpeedR = 0;
   }
   
   // Stop the drive base if a drive msg hasn't been recieved recently
   if (millis() - lastDriveMsgTime > DRIVE_CMD_WATCHDOG_MS) {
     driveMotors(0,0);
     driveAux(0, 0, 0);
     shadowStatus.noDriveMsgStopped = true;
   } else {
     //speedControlMotors();
     shadowStatus.noDriveMsgStopped = false;
   }
}

void publishToROS() {
   encs_msg.encoders[0].travel = prevRightDist;
   encs_msg.encoders[1].travel = prevLeftDist;
   encs_msg.encoders[0].speed = printSpeedR / (ROS_UPDATE_PERIOD/SPEED_CTRL_PERIOD);
   encs_msg.encoders[1].speed = printSpeedL / (ROS_UPDATE_PERIOD/SPEED_CTRL_PERIOD);
   enc_pub.publish(&encs_msg);
   
   // Publish status data
   shadowStatus.batteryVoltage = getBatteryVoltage();
   shadowStatus.eStopped = digitalRead(E_STOP_PIN);
   status_pub.publish(&shadowStatus);
   
   // See if the PID values have changed
   //nh.getParam("shadowP", &shadowP);
   //nh.getParam("maxSpeed", &maxSpeed);
   
   nh.spinOnce();
}

void encRInc() {
  encCountR += digitalRead(ENC_R_B) == LOW ? 1 : -1;
  endPeriodR = micros();
}

void encLInc() {
  encCountL += digitalRead(ENC_L_B) == LOW ? -1 : 1;
  endPeriodL = micros();
}

float getBatteryVoltage() {
  return analogRead(BAT_VOLT_PIN)/1024.0 * 15;
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
  leftMotorTarget = leftSpeed;
  rightMotorTarget = rightSpeed;
}

void speedControlMotors() {
    leftMotorPower += (leftMotorTarget - leftMotorSpeed) * shadowP;
    rightMotorPower += (rightMotorTarget - rightMotorSpeed) * shadowP;
    leftMotorPower = leftMotorPower > maxSpeed ? maxSpeed : leftMotorPower;
    leftMotorPower = leftMotorPower < -maxSpeed ? -maxSpeed : leftMotorPower;
    rightMotorPower = rightMotorPower > maxSpeed ? maxSpeed : rightMotorPower;
    rightMotorPower = rightMotorPower < -maxSpeed ? -maxSpeed : rightMotorPower;
    driveMotors(leftMotorPower, rightMotorPower);
}

void driveMotors(float leftSpeed, float rightSpeed) {
  int servoValL = leftSpeed * 500 + 1500;
  int servoValR = -rightSpeed * 500 + 1500;
  leftDrive.writeMicroseconds(servoValL);
  rightDrive.writeMicroseconds(servoValR);
}

void driveAux(float panSpeed, float tiltSpeed, float bucketSpeed) {
  int servoValP = panSpeed * 500 + 1500;
  int servoValT = tiltSpeed * 500 + 1500;
  int servoValB = bucketSpeed * 500 + 1500;
  camPan.writeMicroseconds(servoValP);
  camTilt.writeMicroseconds(servoValT);
  bucket.writeMicroseconds(servoValB);
}


