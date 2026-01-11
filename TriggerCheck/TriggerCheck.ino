<<<<<<< HEAD

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
=======
const int triggerPin = 2; 

void setup() {
  Serial.begin(115200);
  
  // Set the digital pin to be an input 
  pinMode(triggerPin, INPUT); 
  
  // ISR and attachInterrupt removed to match unedited trigger system logic
  Serial.println("System ready. Waiting for trigger on Pin D2..."); 
}

void loop() {
  // Check trigger state directly via digitalRead 
  if (digitalRead(triggerPin) == HIGH) {
    Serial.println("TRIGGER DETECTED!"); 
    
    // Logic for peak detection/processing would go here in the unedited code 
    // Wait for trigger pin to return to LOW before allowing the next cycle 
    // This ensures the "exact same" behavior as the original while(trigger_active)
    while(digitalRead(triggerPin) == HIGH); 
  }
  else {
    Serial.println("nope");
  }
}
>>>>>>> 2b0e327 (Home computer update)
