#define TOP  13
#define BOTTOM 12

void setup() {
   pinMode(TOP, OUTPUT);
   pinMode(BOTTOM, OUTPUT);
}

void loop() {

   topMode = Serial.parseFloat();  
   bottomMode = Serial.parseFloat();
   
   if(topMode > 1500) {
     turnOn(top);
   }  
   

}

void turnOn(int port) {
  digitalWrite(port, HIGHT);
  
}

void turnOff(int port) {
  digitalWrite(port, LOW);
  
}
