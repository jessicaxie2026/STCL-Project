  
const int analogPin = A4;       // Connect PCB output to A0
const int sampleInterval = 1000; // Microseconds between samples


void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Due suppo rts 12-bit ADC
}

void loop() { 
  int currentValue = analogRead(analogPin); 
  Serial.println(currentValue);
}
