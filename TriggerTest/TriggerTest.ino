// Raw signal with one laser and trigger system

#define pin_input1 A2 
#define dpin_in 3     // Trigger input (polling)
#define dpin_out 6    // Output for function generator

void setup() {
  Serial.begin(250000);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT);
  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
}

void loop() {
  bool lock = digitalRead(dpin_in);
  if (lock) {
    digitalWrite(dpin_out, HIGH);
    int raw_signal = analogRead(pin_input1);
    Serial.println(raw_signal);
    digitalWrite(dpin_out, LOW); //Change the trigger state back to LOW
  } else {
    Serial.println("Lock disengaged");
  }
}
