// --- Configuration and Thresholds ---
#define High_threshold1 850 
#define Low_threshold1 850 

#define pin_input1 A2
#define dpin_in 2       // Pin for external trigger
#define arraysize 2000 

// --- Global Variables ---
unsigned long t01, start_time, end_time, time_peak;
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH;
volatile bool trigger_active = false;

void setup() {
  Serial.begin(115200); 
  analogReadResolution(12); 
  
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
        time_peak = micros();
        int i = 0;
        
        // Capture peak data into array
        do {
          if (i < arraysize) signalarray[i] = value1;
          value1 = analogRead(pin_input1); 
          i++;
        } while (value1 > Low_threshold1);
        
        end_time = micros();
        
        // Calculate exact peak time using Savitzky-Golay filter
        // time_peak is the start of capture; peakfinder adds the offset to the center
        t01 = (time_peak - start_time) + peakfinder(i, end_time - time_peak); 
        peak_captured = true;
      }
    }

    // --- Output Results ---
    if (peak_captured) {
      Serial.print("Peak Time (us): "); 
      Serial.println(t01);
    } else {
      Serial.println("Error: No peak detected within timeout.");
    }

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
    dsignalarray[j] = (6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - 
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]);

    // Zero-crossing detection for the derivative (peak center)
    if (dsignalarray[j] <= 0 && flag) { 
      flag = LOW;
      // Linear interpolation for sub-sample precision 
      return (unsigned long)((j + (double)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt);
    }
  }
  return 0;
}
