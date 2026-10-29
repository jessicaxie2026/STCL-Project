#define pin DAC0
  

void setup() {
  analogWriteResolution(12); // Due suppo rts 12-bit ADC
}



void loop() {
  analogWrite(pin, 1700);

}
