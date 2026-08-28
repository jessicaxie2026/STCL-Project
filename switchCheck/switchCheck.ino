// Define the pin connected to your switch
const int switchPin = 8; // <-- Change this to your actual switch pin number

int lastSwitchState = HIGH; // Default to HIGH due to INPUT_PULLUP

void setup() {
  Serial.begin(115200);
  
  // Set the pin as INPUT_PULLUP
  pinMode(switchPin, INPUT_PULLUP);
  
  Serial.println("--- Switch State Test Initialized ---");
  Serial.print("Initial Pin State: ");
  Serial.println(digitalRead(switchPin) == LOW ? "LOW" : "HIGH");
  Serial.println("Flip the physical switch to test...");
}

void loop() {
  int currentSwitchState = digitalRead(switchPin);

  // Check if the physical switch state has changed
  if (currentSwitchState != lastSwitchState) {
    
    if (currentSwitchState == LOW) {
      Serial.println("Switch Closed: Pin is LOW (Successfully grounded)");
    } else {
      Serial.println("Switch Open: Pin is HIGH (Successfully pulled up to 3.3V)");
    }
    
    // Tiny delay to filter out mechanical contact bounce (debouncing)
    delay(50); 
    
    // Update the saved state for the next comparison
    lastSwitchState = currentSwitchState;
  }
}
