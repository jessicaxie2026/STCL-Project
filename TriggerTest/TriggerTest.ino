#define pin_input1 A2 // input signal from PD1
#define dpin_in 2

volatile bool trigger_active = false;

void setup() {
  Serial.begin(115200);
  
  // Keep your high resolution settings if using a board like a Due or Zero
  analogWriteResolution(12);
  analogReadResolution(12);
  
  pinMode(dpin_in, INPUT);
  
  // Set up the interrupt to track the trigger state [cite: 2, 3]
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  trigger_active = digitalRead(dpin_in);
}

void loop() {
  if (trigger_active) {
    // When trigger is active, read the actual laser signal [cite: 4, 7]
    int raw_signal = analogRead(pin_input1);
    Serial.println(raw_signal);
  } else {
    // When trigger is inactive, output 0 as requested
    Serial.println(0);
  }
  
  // Small delay to prevent overwhelming the Serial buffer (optional)
  // delayMicroseconds(100); 
}

void triggerISR() {
  // Updates the trigger state immediately when the pin changes [cite: 2, 14]
  trigger_active = digitalRead(dpin_in);
}
