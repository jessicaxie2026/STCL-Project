// --- Configuration and Thresholds ---
// Reference laser (adjusted for lower amplitude peaks)
#define High_threshold1 1250
#define Low_threshold1 1200

// Slave laser (adjusted for lower amplitude peaks)
#define High_threshold2 900
#define Low_threshold2 885

#define pin_input1 A8 
#define dpin_in 8       // Manual lock switch
#define dpin_out 12     // Trigger input from Function Gen
#define arraysize 2000
#define alpha2_ref 0.50

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, time_peak, tstartsweep;
unsigned long period = 100; // 55ms safety margin for 20Hz (50ms) ramp
int signalarray[arraysize];
bool sweep_active = false;
bool prev_trigger_state = false;
int counter = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Serial.println("System Ready: Reading Big, Small, (skip) Small, Big sequence.");
}

void loop() {
  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  // Detect Trigger Start (Falling Edge)
  if (!sweep_active) {
    if (!prev_trigger_state && trigger_now) {
      start_time = micros();
      tstartsweep = millis();
      counter = 0;
      sweep_active = true;
    }
    prev_trigger_state = trigger_now;
    return;
  }

  // Safety Timeout: Reset if sweep takes too long
  if (millis() - tstartsweep > period) {
    if (counter == 0) Serial.println("Missing peak: reference peak 1");
    else if (counter == 1) Serial.println("Missing peak: slave peak");
    else if (counter == 3) Serial.println("Missing peak: reference peak 2");
    else Serial.println("Missing peak: unknown peak");
    sweep_active = false;
    counter = 0;
    return;
  }

  int sample = analogRead(pin_input1);

  // --- STATE MACHINE WITH TIMING GATES ---

  // PEAK 0: First Big Peak (Master 1)
  if (counter == 0 && sample > High_threshold1) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold1 && i < arraysize);
    
    t01 = time_peak - start_time + peakfinder(i, micros() - time_peak);
    counter++; 
  }

  // PEAK 1: First Small Peak (Slave)
  // GATE: Only accept if found within 10ms of the first big peak
  else if (counter == 1 && sample > High_threshold2) {
    time_peak = micros();
    unsigned long current_offset = time_peak - start_time;
    
    if (current_offset > t01 && (current_offset - t01) < 10000) {
      int i = 0;
      do {
        if (i < arraysize) signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > Low_threshold2 && i < arraysize);
      
      t2 = current_offset + peakfinder(i, micros() - time_peak);
      counter++;
    } else {
      Serial.println("Missing peak: slave peak");
      sweep_active = false;
      counter = 0;
    }
  }

  // PEAK 2: Second Small Peak (SKIP)
  else if (counter == 2 && sample > High_threshold2) {
    int i = 0;
    do {
      sample = analogRead(pin_input1);
    } while (sample > Low_threshold2 && i < arraysize);
    counter++; 
  }

  // PEAK 3: Second Big Peak (Master 2)
  else if (counter == 3 && sample > High_threshold1) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold1 && i < arraysize);
    
    t02 = time_peak - start_time + peakfinder(i, micros() - time_peak);
    counter++;
  } else if (counter == 3) {
    Serial.println("Missing peak: reference peak 2");
    sweep_active = false;
    counter = 0;
  }

  // --- VALIDATION AND OUTPUT ---
  if (counter >= 4) {
    sweep_active = false;
    
    // ALPHA GUARD: Verify the peak is between the two master peaks
    if (t2 > t01 && t2 < t02) {
      double alpha2 = (double)(t2 - t01) / (t02 - t01);
      double lock_error = alpha2_ref - alpha2;

      Serial.print(t01); Serial.print(", ");
      Serial.print(t2); Serial.print(", ");
      Serial.print(t02); Serial.print(", Lock Error: ");
      Serial.println(lock_error, 6);
    } else {
      // Discard glitchy sweep
      counter = 0;
      Serial.println("Missing peak: invalid sequence");
    }
  }

  prev_trigger_state = trigger_now;
}

// Savitzky-Golay Peak Finder
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  int prev_d = 0;
  for (int j = 6; j < (number - 7); j++) {
    int current_d = (6 * signalarray[j+6] + 5 * signalarray[j+5] + 4 * signalarray[j+4] + 
                     3 * signalarray[j+3] + 2 * signalarray[j+2] + signalarray[j+1] - 
                     signalarray[j-1] - 2 * signalarray[j-2] - 3 * signalarray[j-3] - 
                     4 * signalarray[j-4] - 5 * signalarray[j-5] - 6 * signalarray[j-6]);
    if (current_d <= 0 && j > 6) {
      return (unsigned long)((j + (double)current_d / (prev_d - current_d)) * dt);
    }
    prev_d = current_d;
  }
  return 0;
}
