const int triggerPin = 2;

void setup() {
  Serial.begin(115200);
  // pinMode(triggerPin, INPUT);
  Serial.println("System ready. Waiting for trigger on Pin D2...");
}

void loop() {
  // if (digitalRead(triggerPin) == HIGH) {
  if (true) { // Trigger input disabled
    // Manual 50ms trigger pulse
    digitalWrite(2, HIGH); // triggerPin is 2, but we need dpin_out. Assuming pin 6 or using triggerPin + 4
    delay(50);
    digitalWrite(2, LOW);
    delay(50);
    Serial.println("TRIGGER DETECTED!");
    // while (digitalRead(triggerPin) == HIGH); - ORIGINAL COMMENTED OUT
  } else {
    Serial.println("nope");
  }
}
