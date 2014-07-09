#include <fuckitMsg.h>
#include <Servo.h>

#define ENCODER_RIGHT_INT 0
#define ENCODER_LEFT_INT 1
#define ENC_R_A 2
#define ENC_R_B 4
#define ENC_L_A 3
#define ENC_L_B 5

#define BAT_VOLT_PIN 0

Servo leftDrive;
Servo rightDrive;

int encCountR = 0;
int encCountL = 0;

byte dirL = 0;
byte dirR = 0;

uint16_t periodL, periodR;

void setup() {
  Serial.begin(115200);
  leftDrive.attach(10);
  rightDrive.attach(9);
  
  attachInterrupt(ENCODER_RIGHT_INT, encRInc, RISING);
  attachInterrupt(ENCODER_LEFT_INT, encLInc, RISING);
  pinMode(ENC_R_B, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(E_STOP_PIN, INPUT);
}

void encRInc() {
  static unsigned long start = 0;
  if (digitalRead(ENC_R_B) == LOW) {
    encCountR++;
    dirR = 1;
  } else {
    encCountR--;
    dirR = 0;
  }
  unsigned long endP = micros();
  periodR = endP - start;
}

void encLInc() {
  static unsigned long start = 0;
  if (digitalRead(ENC_L_B) == LOW) {
    encCountL++;
    dirL = 1;
  } else {
    encCountL--;
    dirL = 0;
  }
  unsigned long endP = micros();
  periodL = endP - start;
}

int16_t cap(int32_t a, int32_t minVal, int32_t maxVal) {
  a = a < minVal ? minVal : a;
  return a > maxVal ? maxVal : a;
}

void magicTimer() {
  static int16_t prevErrorR, prevErrorL = 0;
  static int32_t iErrorL, iErrorR = 0;
  int16_t freqL = (MIN_PULSE_PERIOD * MAX_INT16) / periodL;
  int16_t freqR = (55 * MAX_INT16) / periodR;
  freqL = dirL ? freqL : -freqL;
  freqR = dirR ? freqR : -freqR;
  
  int32_t errorL = targetL - freqL;
  int32_t errorR = targetR - freqR;
  
  iErrorL += errorL;
  iErrorR += errorR;
  iErrorL = cap(iErrorL, MAX_INT16, -MAX_INT16);
  iErrorR = cap(iErrorR, MAX_INT16, -MAX_INT16);
  
  int32_t dErrorL = errorL - prevErrorL;
  int32_t dErrorR = errorR - prevErrorR;
  
  int32_t sIErrorL = (iErrorL * I) >> 7;
  int32_t sIErrorR = (iErrorR * I) >> 7;
  errorL = (errorL * P) >> 7;
  errorR = (errorR * P) >> 7;
  dErrorL = (dErrorL * D) >> 7;
  dErrorR = (dErrorR * D) >> 7;
  
  int32_t totalL = errorL + dErrorL + sIErrorL;
  int32_t totalR = errorR + dErrorR + sIErrorR;
  totalL = cap(totalL, MAX_INT16, -MAX_INT16);
  totalR = cap(totalR, MAX_INT16, -MAX_INT16);
  
  driveMotors(totalL, totalR);
}
  
uint8_t getBatteryVoltage() {
  return analogRead(BAT_VOLT_PIN) >> 2;
}

void driveMotors(int16_t leftSpeed, int16_t rightSpeed) {
  int servoValL = leftSpeed * 500 / MAX_INT16 + 1500;
  int servoValR = -rightSpeed * 500 / MAX_INT16 + 1500;
  leftDrive.writeMicroseconds(servoValL);
  rightDrive.writeMicroseconds(servoValR);
}

struct arduinoRxPkt completeRx;
void loop() {
  int i = 0;
  struct arduinoRxPkt rx;
  struct sabreRxPkt tx;
  if (Serial.isAvailable()) {
    (uint8_t *)arduinoRxPkt[i] = Serial.readByte();
    i++;
    if (i == sizeof(struct arduinoRxPkt)) {
      completeRx = rx;
      tx.encCountL = encCountL;
      tx.encCountR = encCountR;
      tx.batteryVoltage = getBatteryVoltage();
      Serial.writeBytes(tx, sizeof(struct sabreRxPkt));
  }
}
