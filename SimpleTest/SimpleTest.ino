#define dpin_in 3     
#define dpin_out 6    


void setup() {
  Serial.begin(250000);
  // TriggerCheck mapping: dpin_in = lock (INPUT_PULLUP), dpin_out = triggerPin (INPUT)
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
}

void loop() {
  if (digitalRead(dpin_in) == LOW) {
    if (digitalRead(dpin_out) == HIGH) {
      Serial.println("TRIGGER DETECTED!");
    } else {
      Serial.println("no trigger");
    }
  } else {
    Serial.println("switch off");
  }
  delay(50);
}
