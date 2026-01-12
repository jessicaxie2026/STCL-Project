// Raw signal with one laser and trigger system

#define pin_input1 A2 
#define dpin_in 3     // Trigger input (polling)
#define dpin_out 6    // Output for function generator

void setup() {
  Serial.begin(250000);
  analogWriteResolution(12);
  analogReadResolution(12);
  // pinMode(dpin_in, INPUT);
  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
}

void loop() {
  // bool lock = digitalRead(dpin_in);
  // if (lock) {
  if (true) { // Trigger input disabled
    // Manual 50ms trigger pulse
    digitalWrite(dpin_out, HIGH);
    delay(50);
    digitalWrite(dpin_out, LOW);
    delay(50);
    // digitalWrite(dpin_out, HIGH); - ORIGINAL COMMENTED OUT
    int raw_signal = analogRead(pin_input1);
    Serial.println(raw_signal);
    // digitalWrite(dpin_out, LOW); - ORIGINAL COMMENTED OUT
  } else {
    Serial.println("Lock disengaged");
  }
}
