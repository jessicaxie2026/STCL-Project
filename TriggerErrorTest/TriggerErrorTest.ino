// --- Configuration and Thresholds ---
#define High_threshold1 500 
#define Low_threshold1 400 
#define High_threshold2 1000 
#define Low_threshold2 700 

#define pin_input1 A2
#define pin_input2 A4 
#define dpin_in 2       // Pin for external trigger
#define arraysize 2000 
#define alpha2_ref 0.50

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, end_time, time_peak;
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH;
double error2, totalT, alpha2;

// Volatile variable shared between ISR and main loop as used in TwoTriggerRead.ino
volatile bool trigger_active = false;

void setup() {
  Serial.begin(115200); 
  analogReadResolution(12); 
  
  // Setup trigger pin and interrupt logic exactly as in TwoTriggerRead.ino
  pinMode(dpin_in, INPUT);
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  
  // Initial check of the trigger state
  trigger_active = digitalRead(dpin_in);
}

void loop() {
  // Use the volatile trigger_active flag updated by the ISR
  if (trigger_active) { 
    start_time = micros(); 
    int counter = 0;
    bool indicator2 = LOW;
    bool sweep_active = HIGH;
    unsigned long tstartsweep = millis();

    while (sweep_active) {
      int value1 = analogRead(pin_input1); 
      int value2 = analogRead(pin_input2); 

      // Timeout safety (50ms)
      if (millis() - tstartsweep > 50) sweep_active = LOW; 

      // Detect reference peaks on Channel 1
      if (value1 > High_threshold1) { 
        time_peak = micros();
        int i = 0;
        do {
          if (i < arraysize) signalarray[i] = value1;
          value1 = analogRead(pin_input1); 
          i++;
        } while (value1 > Low_threshold1);
        
        end_time = micros();
        unsigned long p_time = time_peak - start_time + peakfinder(i, end_time - time_peak); 

        if (counter == 0) t01 = p_time;
        if (counter == 2) t02 = p_time;
        counter++;
      }

      // Detect signal peak on Channel 2 (between ref peaks)
      if (counter == 1 && value2 > High_threshold2 && !indicator2) { 
        indicator2 = HIGH;
        time_peak = micros();
        int i = 0;
        do {
          if (i < arraysize) signalarray[i] = value2;
          value2 = analogRead(pin_input2);
          i++;
        } while (value2 > Low_threshold2);
        
        end_time = micros();
        t2 = time_peak - start_time + peakfinder(i, end_time - time_peak); 
        counter++;
      }
      
      if (counter >= 3) sweep_active = LOW; 
    }

    // --- Output Results ---
    if (counter >= 3) {
      error2 = (double)t2 - t01; 
      totalT = (double)t02 - t01; 
      alpha2 = error2 / totalT; 
      double lock_error = alpha2_ref - alpha2; 

      Serial.print("Peak01: "); Serial.print(t01);
      Serial.print(" | Peak02: "); Serial.print(t02);
      Serial.print(" | SignalPeak: "); Serial.print(t2);
      Serial.print(" | Lock Error: "); Serial.println(lock_error, 6);
    } else {
      Serial.println("Error: Failed to capture all 3 peaks.");
    }

    // Wait for trigger_active to become false (via ISR) before next cycle
    while(trigger_active); 
  }
}

// ISR exactly as implemented in TwoTriggerRead.ino
void triggerISR() {
  trigger_active = digitalRead(dpin_in);
}

// Savitzky-Golay Filter for peak center detection
unsigned long peakfinder(int number, unsigned long duration) { 
  unsigned long dt = duration / number; 
  flag = HIGH;
  
  for (int j = 6; j < (number - 7); j++) { 
    // 13-point SG derivative filter 
    dsignalarray[j] = (6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - 
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]);

    if (dsignalarray[j] <= 0 && flag) { 
      flag = LOW;
      // Linear interpolation for sub-sample precision 
      return (j + (double)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;
    }
  }
  return 0;
}
