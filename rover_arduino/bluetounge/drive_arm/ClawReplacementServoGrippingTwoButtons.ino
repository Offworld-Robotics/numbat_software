#include <Servo.h> 
#define BAUD_RATE 9600
#define MIN_BYTES_AVAILABLE_TO_READ 5

#define OPEN  0
#define STOP  1
#define CLOSE 2
Servo myservo; 

//int button1 = 6;
//int button2 = 7;

void setup() 
{ 
  myservo.attach(9); 
  Serial.begin(9600);
}
void loop() {
  if(Serial.available() >= MIN_BYTES_AVAILABLE_TO_READ){
    int state = Serial.parseInt();
    if (state==OPEN){
      myservo.write(87);
    } else if (state==CLOSE){
      myservo.write(97);
    } else {
      myservo.write(92);
    }
  }
} 

