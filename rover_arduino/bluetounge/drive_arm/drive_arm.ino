int servopin = 4; 
int pulse = 1500; 
void setup() {
  pinMode(servopin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(servopin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(servopin, LOW);
  delay(20);
}
