const int laserPin1 = A2;        // First laser connected to A2
const int laserPin2 = A4;        // Second laser connected to A4
const int dpin_in = 2;           // External trigger pin (from TriggerRun.ino)
const int dpin_out = 6;          // Digital output for triggering function generator
const int sampleInterval = 10000; // Microseconds between samples

// Trigger state will be polled from `dpin_in`

void setup() {
  Serial.begin(115200);
  
  // Set resolution to match TriggerRun.ino setup
  analogReadResolution(12);      
  
  // Setup trigger pin for polling
  // pinMode(dpin_in, INPUT);
  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
}

void loop() {
  // Poll the trigger pin (two-pin system: dpin_in)
  // if (digitalRead(dpin_in)) {
  if (true) { // Trigger input disabled
    // Manual 50ms trigger pulse
    digitalWrite(dpin_out, HIGH);
    delay(50);
    digitalWrite(dpin_out, LOW);
    delay(50);
    
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
