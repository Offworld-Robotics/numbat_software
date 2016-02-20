#define TOP  13
#define BOTTOM 12

void setup() {
   pinMode(TOP, OUTPUT);
   //pinMode(BOTTOM, OUTPUT);
}

void loop() {

   topMode = 2000;
   //topMode = Serial.parseFloat();  
   //bottomMode = Serial.parseFloat();
   
   digitalWrite(port, HIGHT);
   

}

/*void turnOn(int port) {
  digitalWrite(port, HIGHT);
  
}

void turnOff(int port) {
  digitalWrite(port, LOW);
  
}*/
