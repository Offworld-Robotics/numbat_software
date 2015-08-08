#include <Servo.h> 
 
Servo myservo; 


#define OPEN  0
#define STOP  1
#define CLOSE 2
#define MIN_BYTES_AVAILABLE_TO_READ 1
//int button1 = 6;
//int button2 = 7;

void setup() 
{ 
  myservo.attach(9);
  Serial.begin(9600);
  myservo.write(92);
}
void loop() { 
  int armState = Serial.parseInt();
  if(armState == OPEN){
    myservo.write(87);
  } else if(armState == CLOSE){
    myservo.write(97);
  } else {
    myservo.write(92);
  }
}
    
      

