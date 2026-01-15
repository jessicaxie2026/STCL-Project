const int triggerPin = 2;

void setup() {
  Serial.begin(115200);
  pinMode(triggerPin, INPUT);
  Serial.println("System ready. Waiting for trigger on Pin D2...");
}

void loop() {
  if (digitalRead(triggerPin) == HIGH) {
    digitalWrite(2, HIGH);
    Serial.println("TRIGGER DETECTED!");
    while (digitalRead(triggerPin) == HIGH);
    digitalWrite(2, LOW); //Change the trigger state back to LOW
  } else {
    Serial.println("nope");
  }
}
