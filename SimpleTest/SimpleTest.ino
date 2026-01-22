#define dpin_in 3     
#define dpin_out 6    


void setup() {
  Serial.begin(250000);
  pinMode(dpin_in, INPUT);
  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
}

void loop() {
    digitalWrite(dpin_out, HIGH);
    delay(50);
    digitalWrite(dpin_out, LOW);
    delay(50);
}
