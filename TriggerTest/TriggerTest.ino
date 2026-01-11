<<<<<<< HEAD
#define pin_input1 A2 // input signal from PD1
#define dpin_in 2

volatile bool trigger_active = false;

void setup() {
  Serial.begin(115200);
  
  // Keep your high resolution settings if using a board like a Due or Zero
  analogWriteResolution(12);
  analogReadResolution(12);
  
  pinMode(dpin_in, INPUT);
  
  // Set up the interrupt to track the trigger state [cite: 2, 3]
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  trigger_active = digitalRead(dpin_in);
}

void loop() {
  if (trigger_active) {
    // When trigger is active, read the actual laser signal [cite: 4, 7]
    int raw_signal = analogRead(pin_input1);
    Serial.println(raw_signal);
  } else {
    // When trigger is inactive, output 0 as requested
    Serial.println(0);
  }
  
  // Small delay to prevent overwhelming the Serial buffer (optional)
  // delayMicroseconds(100); 
}

void triggerISR() {
  // Updates the trigger state immediately when the pin changes [cite: 2, 14]
  trigger_active = digitalRead(dpin_in);
}
=======
// Raw signal with one laser and trigger system

#define pin_input1 A2 
#define dpin_in 3     // Updated to match UneditedCode 
#define dpin_out 6    // Added output for function generator 

void setup() {
  Serial.begin(250000); // Updated to match UneditedCode baud rate 
  
  analogWriteResolution(12);
  analogReadResolution(12);
  
  pinMode(dpin_in, INPUT);  
  pinMode(dpin_out, OUTPUT); // 
  digitalWrite(dpin_out, LOW); // Initialize trigger as LOW 
}

void loop() {
  // Read the lock status (dpin_in) 
  bool lock = digitalRead(dpin_in);

  if (lock) {
    // 1. Signal the start of the sweep 
    digitalWrite(dpin_out, HIGH);
    
    // 2. Perform the analog reading
    int raw_signal = analogRead(pin_input1);
    Serial.println(raw_signal);

    // 3. Signal the end of the sweep cycle 
    digitalWrite(dpin_out, LOW);
    
  } else {
    // Output matches the "disengaged" state of the unedited code 
    Serial.println("Lock disengaged");
  }
}
>>>>>>> 2b0e327 (Home computer update)
