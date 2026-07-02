// Raw signal with one laser and trigger system

#define pin_input1 A2 
#define dpin_in 8   // Trigger input (polling)
#define dpin_out 10    // Output for function generator

// Using TriggerCheck-style polling: dpin_in acts as lock (INPUT_PULLUP)

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
}

void loop() {
  if (digitalRead(dpin_in) == LOW) {
    if (digitalRead(dpin_out) == HIGH) {
      int raw_signal = analogRead(pin_input1);
      Serial.println(raw_signal);
    } else {
      Serial.println(0);
    }
  } else {
    Serial.println("switch off");
  }
}
