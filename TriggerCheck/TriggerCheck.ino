const int triggerPin = 6;  
const int lock = 4;      

void setup() {
  Serial.begin(115200); 
  
  pinMode(triggerPin, INPUT); 
  
  pinMode(lock, INPUT_PULLUP); 

  Serial.println("System ready. Waiting for trigger on Pin D3..."); 
}

void loop() {
  if (digitalRead(lock) == LOW) { 
    
    // Check the external triggerPin 
    if (digitalRead(triggerPin) == HIGH) { 
        Serial.println("TRIGGER DETECTED!"); 
        delayMicroseconds(1000); 
    }
    
    if (digitalRead(triggerPin) == LOW) { 
      Serial.println("no trigger"); 
      delayMicroseconds(1000); 
    }
    
  } 
  
  // If the SPDT switch is flipped away from Ground (HIGH)
  if (digitalRead(lock) == HIGH) { 
    Serial.println("switch off"); 
    delayMicroseconds(1000); 
  }
}
