#include <Servo.h>
#define BAUD_RATE 9600
#define MIN_BYTES_AVAILABLE_TO_READ 5
// MIN_BYTES_AVAILABLE_TO_READ should equal the minimum expexted length of a command
//  currently actuatorNumber (1 char == 1 byte) + space/newline (1 byte) + position (4 bytes) = 6 bytes
#define PIN_TOP 13
#define PIN_BOTTOM 12

#define TOP_ACTUATOR 0
#define BOTTOM_ACTUATOR 1

// SERVO VALUES DEALS WITH POSITION NOT VELOCITY
#define FULL_EXTENSION 1700
//1700
#define FULL_RETRACTION 1200
//1250
Servo top, bottom;

void setup() {
  top.attach(PIN_TOP, FULL_RETRACTION, FULL_EXTENSION);
  bottom.attach(PIN_BOTTOM, FULL_RETRACTION, FULL_EXTENSION);
  bottom.writeMicroseconds(1500);
  top.writeMicroseconds(1500);
  Serial.begin(9600);
}

void loop() {
  
  /* 
   * Serial.available() returns the number of bytes available to read from the serial connection
   *   note 1 character is 1 byte, so could check for when a whole command is available before reading 
  */
  if(Serial.available() >= MIN_BYTES_AVAILABLE_TO_READ){
    /*
      Command Format 
      
      (actuatorNumber) (position), ie: 0 1400
      
      where actuator number is 
        TOP_ACTUATOR ie 0  
        or
        BOTTOM_ACTUATOR 1
        
        and position is an integer between FULL_RETRACTION and FULL_EXTENSION
     */ 
    int actuator = Serial.parseInt();
    int pos = Serial.parseInt();
    int valid = 0;
    Servo dest;
    if (actuator == TOP_ACTUATOR) {
        dest = top;
        valid = 1;
    } else if (actuator == BOTTOM_ACTUATOR) {
        dest = bottom;
        valid = 1;
    }
    
    if (valid) {
        if (pos < FULL_RETRACTION) {
           pos = FULL_RETRACTION; 
        }
        if (pos > FULL_EXTENSION) {
           pos = FULL_EXTENSION; 
        }
        Serial.println(actuator);
        Serial.println(pos);
        // higher value = more extension
        dest.writeMicroseconds(pos);
    }
  }  
  
  
}

