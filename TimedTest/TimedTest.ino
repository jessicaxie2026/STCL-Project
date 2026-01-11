<<<<<<< HEAD
=======
// one laser code peak time
>>>>>>> 2b0e327 (Home computer update)
// --- Configuration and Thresholds ---
#define High_threshold1 850 
#define Low_threshold1 850 

#define pin_input1 A2
<<<<<<< HEAD
#define dpin_in 2       // Pin for external trigger
#define arraysize 2000 

// --- Global Variables ---
unsigned long t01, start_time, end_time, time_peak;
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH;
volatile bool trigger_active = false;
=======
#define dpin_in 3       // Input pin for external trigger/lock
#define dpin_out 6      // Digital output for triggering function generator
#define arraysize 2000  // Size of the array for storing peak data

// --- Global Variables ---
unsigned long t01, start_time, end_time, time_peak, tstartsweep;
unsigned long period = 50; // Sweep period limit in ms
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH; // Flag for detecting peak position
bool trigger;     // Sweep control flag
>>>>>>> 2b0e327 (Home computer update)

void setup() {
  Serial.begin(115200); 
  analogReadResolution(12); 
  
<<<<<<< HEAD
  pinMode(dpin_in, INPUT);
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  
  // Initial check of the trigger state
  trigger_active = digitalRead(dpin_in);
}

void loop() {
  // Use the volatile trigger_active flag updated by the ISR
  if (trigger_active) { 
    start_time = micros();
    bool peak_captured = false;
    unsigned long tstartsweep = millis();

    // Loop until one peak is found or 50ms timeout occurs
    while (!peak_captured) {
      int value1 = analogRead(pin_input1); 

      // Timeout safety (50ms)
      if (millis() - tstartsweep > 50) break;

      // Detect peak on Channel 1
      if (value1 > High_threshold1) { 
=======
  pinMode(dpin_in, INPUT);   
  pinMode(dpin_out, OUTPUT); 
  digitalWrite(dpin_out, LOW); 
}

void loop() {
  // Check if the lock/trigger pin is HIGH to start the cycle
  bool lock = digitalRead(dpin_in);

  if (lock) {
    start_time = micros();      
    digitalWrite(dpin_out, HIGH); // Signal start of the sweep
    trigger = HIGH;
    tstartsweep = millis();     
    bool peak_captured = false;

    // Loop until sweep period expires (matches UneditedCode logic)
    do {
      int value1 = analogRead(pin_input1);
      unsigned long timenow = millis();

      // Timeout safety
      if (timenow - tstartsweep > period) { 
        trigger = LOW;
      }

      // Detect peak on Channel 1
      if (value1 > High_threshold1 && !peak_captured) { 
>>>>>>> 2b0e327 (Home computer update)
        time_peak = micros();
        int i = 0;
        
        // Capture peak data into array
        do {
          if (i < arraysize) signalarray[i] = value1;
<<<<<<< HEAD
          value1 = analogRead(pin_input1); 
=======
          value1 = analogRead(pin_input1);
>>>>>>> 2b0e327 (Home computer update)
          i++;
        } while (value1 > Low_threshold1);
        
        end_time = micros();
<<<<<<< HEAD
        
        // Calculate exact peak time using Savitzky-Golay filter
        // time_peak is the start of capture; peakfinder adds the offset to the center
        t01 = (time_peak - start_time) + peakfinder(i, end_time - time_peak); 
        peak_captured = true;
      }
    }

    // --- Output Results ---
    if (peak_captured) {
      Serial.print("Peak Time (us): "); 
=======

        // Calculate exact peak time using SG filter
        t01 = (time_peak - start_time) + peakfinder(i, end_time - time_peak);
        peak_captured = true;
      }
    } while (trigger == HIGH);

    // --- Output Results ---
    if (peak_captured) {
      Serial.print("Peak Time (us): ");
>>>>>>> 2b0e327 (Home computer update)
      Serial.println(t01);
    } else {
      Serial.println("Error: No peak detected within timeout.");
    }

<<<<<<< HEAD
    // Wait for trigger_active to become false (via ISR) before next cycle
    while(trigger_active);
  }
}

// ISR to monitor external trigger state
void triggerISR() {
  trigger_active = digitalRead(dpin_in);
}

// Savitzky-Golay Filter for peak center detection
unsigned long peakfinder(int number, unsigned long duration) { 
  if (number < 14) return 0; // Ensure enough points for the 13-point filter
  
  unsigned long dt = duration / number;
  flag = HIGH;
  
  for (int j = 6; j < (number - 7); j++) { 
    // 13-point SG derivative filter 
=======
    digitalWrite(dpin_out, LOW); // Reset trigger state for next cycle
    
  } else {
    Serial.println("Lock disengaged");
  }
}

// Savitzky-Golay Filter for peak center detection
unsigned long peakfinder(int number, unsigned long duration) { 
  if (number < 14) return 0;
  
  unsigned long dt = duration / number;
  flag = HIGH;

  for (int j = 6; j < (number - 7); j++) { 
    // 13-point Savitzky-Golay derivative filter
>>>>>>> 2b0e327 (Home computer update)
    dsignalarray[j] = (6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - 
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]);

<<<<<<< HEAD
    // Zero-crossing detection for the derivative (peak center)
    if (dsignalarray[j] <= 0 && flag) { 
      flag = LOW;
      // Linear interpolation for sub-sample precision 
=======
    // Zero-crossing detection
    if (dsignalarray[j] <= 0 && flag) { 
      flag = LOW;
>>>>>>> 2b0e327 (Home computer update)
      return (unsigned long)((j + (double)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt);
    }
  }
  return 0;
<<<<<<< HEAD
}
=======
}
>>>>>>> 2b0e327 (Home computer update)
