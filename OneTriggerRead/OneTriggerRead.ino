const int laserPin1 = A4;         // First laser connected to A8
const int dpin_in = 8;            // Manual switch pin (INPUT_PULLUP)
const int dpin_out = 11;          // External trigger pin (from function generator)

void setup() {
  Serial.begin(115200);
  
  // Set resolution to 12-bit (0 - 4095)
  analogReadResolution(12);      
  
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
}

void loop() {
  if (digitalRead(dpin_in) == LOW) {
    int value1 = analogRead(laserPin1);
    Serial.print(value1);
    Serial.print(","); 
    if (digitalRead(dpin_out) == HIGH) {
      Serial.println(2500); 
    } else {
      Serial.println(1500); 
    }
    
  } else {
    Serial.print(0);
    Serial.print(",");
    Serial.println(0);
  }
}
