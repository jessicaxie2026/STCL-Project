  
const int analogPin = A2 ;       // Connect PCB output to A0
const int sampleInterval = 1000; // Microseconds between samples





void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Due suppo rts 12-bit ADC
  ADC->ADC_MR |= 0x80;  // Use FREERUN mode for faster consecutive reads
  
  // Optional: Set a faster tracking time for high-frequency peaks
  ADC->ADC_MR &= ~(0xF << 24); // Clear tracking time
  ADC->ADC_MR |= (0x1 << 24);  // Set tracking time to minimum
}

void loop() {
  delayMicroseconds(10000);  
  int currentValue = analogRead(analogPin); 
  
}
