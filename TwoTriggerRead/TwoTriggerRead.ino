const int laserPin1 = A2;        // First laser connected to A2
const int laserPin2 = A4;        // Second laser connected to A4
const int dpin_in = 8;           // External trigger pin (from TriggerRun.ino)
const int dpin_out = 10;          // Digital output for triggering function generator
const int sampleInterval = 10000; // Microseconds between samples

// Using TriggerCheck-style polling: dpin_in acts as lock (INPUT_PULLUP)

void setup() {
  Serial.begin(115200);
  
  // Set resolution to match TriggerRun.ino setup
  analogReadResolution(12);      
  
  // Setup trigger pin for polling
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
}

void loop() {
  if (digitalRead(dpin_in) == LOW) {
    if (digitalRead(dpin_out) == HIGH) {
      int value1 = analogRead(laserPin1);
      int value2 = analogRead(laserPin2);
      Serial.print(value1);
      Serial.print(",");
      Serial.println(value2);
    } else {
      Serial.print(0);
      Serial.print(",");
      Serial.println(0);
    }
  } else {
    Serial.println("switch off");
  }
}
