  
const int analogPin = A2 ;       // Connect PCB output to A0
const int sampleInterval = 1000; // Microseconds between samples





void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Due suppo rts 12-bit ADC
}

void loop() {
  delayMicroseconds(10000);  
  int currentValue = analogRead(analogPin); 
  Serial.println(currentValue);
}
