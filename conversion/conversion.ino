#define pin DAC0

void setup() {
  analogWriteResolution(12); // Must be set to 12-bit for Due [4, 5]
}

void loop() {

  analogWrite(pin, 2288);  

}
