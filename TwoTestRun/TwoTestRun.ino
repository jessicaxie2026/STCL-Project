const int signalPin = A8;        // Single peak signal input
const int sampleInterval = 10000; // Microseconds between samples 

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);      // Use 12-bit resolution for higher precision 
}

void loop() {
  // Read both laser signals
  int value1 = analogRead(laserPin1); 
  int value2 = analogRead(laserPin2); 

  // Output values separated by a comma for Serial Plotter compatibility
  Serial.print(value1);
  Serial.print(",");
  Serial.println(value2);
    Serial.print(",");

  delayMicroseconds(sampleInterval);
}
