// Raw signal with one laser and trigger system

#define pin_input1 A2 
#define dpin_in 3     // Trigger input (polling)
#define dpin_out 6    // Output for function generator

volatile bool trigger_active = false;

void setup() {
  Serial.begin(250000);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT);
  pinMode(dpin_out, INPUT);
  attachInterrupt(digitalPinToInterrupt(dpin_out), triggerISR, CHANGE);
}

void loop() {
  bool lock = digitalRead(dpin_in);
  if (lock) {
    if (trigger_active) {
      int raw_signal = analogRead(pin_input1);
      Serial.println(raw_signal);
      while (trigger_active) {}
    }
  } else {
    Serial.println("Lock disengaged");
  }
}

void triggerISR() {
  trigger_active = digitalRead(dpin_out);
}
