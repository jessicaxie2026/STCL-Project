const int laserPin1 = A2;        // First laser connected to A2
const int laserPin2 = A4;        // Second laser connected to A4
const int dpin_in = 2;           // External trigger pin (from TriggerRun.ino)
const int sampleInterval = 10000; // Microseconds between samples

// Use volatile for variables shared between ISR and main loop
volatile bool trigger_active = false; 

void setup() {
  Serial.begin(115200);
  
  // Set resolution to match TriggerRun.ino setup
  analogReadResolution(12);      
  
  // Setup trigger pin and interrupt logic
  pinMode(dpin_in, INPUT);
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  
  // Initial check of the trigger state
  trigger_active = digitalRead(dpin_in);
}

void loop() {
  if (trigger_active) {
    // When trigger is High: output the signal from both lasers
    int value1 = analogRead(laserPin1);
    int value2 = analogRead(laserPin2);

    Serial.print(value1);
    Serial.println(value2);
  } else {
    // When trigger is Low: output 0
    Serial.println(0);
  }

  delayMicroseconds(sampleInterval);
}

// ISR exactly as implemented in TriggerRun.ino
void triggerISR() {
  trigger_active = digitalRead(dpin_in);
}
