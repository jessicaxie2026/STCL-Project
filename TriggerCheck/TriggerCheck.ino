
const int triggerPin = 2; 
volatile bool trigger_detected = false; 

void triggerDetected() {
  trigger_detected = true;
}


void setup() {
  Serial.begin(115200);
  
  // Set the digital pin to be an input
  pinMode(triggerPin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(triggerPin), triggerDetected, RISING);
  Serial.println("System ready. Waiting for trigger on Pin D2...");
}

void loop() {
  if (trigger_detected) {
    Serial.println("TRIGGER DETECTED!");
    trigger_detected = false;
  }
  else {
    Serial.println("nope");
  }
  
}
