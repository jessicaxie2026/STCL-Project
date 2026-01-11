
const int triggerPin = 2; 

void setup() {
  Serial.begin(115200);
  
  // Set the digital pin to be an input
  pinMode(triggerPin, INPUT); 

  Serial.println("Monitoring Digital Pin D2 (Trigger Input)...");
  Serial.println("0 = LOW (0V), 1 = HIGH (3.3V)");
}

void loop() {
  int currentValue = digitalRead(triggerPin); 

  Serial.println(currentValue);
  
  delayMicroseconds(20); 
}
