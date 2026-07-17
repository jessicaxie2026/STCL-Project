const int signalPin = A8;        // Single peak signal input
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
      int value = analogRead(signalPin);
      Serial.println(value);
    } else {
      Serial.println(0);
    }
  } else {
    Serial.println("switch off");
  }
}
