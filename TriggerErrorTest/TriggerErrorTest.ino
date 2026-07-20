// --- Configuration and Thresholds ---
// Reference laser (Big Peaks) thresholds
#define High_threshold1 2950 
#define Low_threshold1 2850

// Slave laser (Small Peaks) thresholds
#define High_threshold2 1400 
#define Low_threshold2 1300

#define pin_input1 A8   // Photodiode signal
#define dpin_in 8       // Manual lock switch
#define dpin_out 12     // Trigger input (Function Gen)
#define arraysize 2000
#define alpha2_ref 0.50

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, time_peak, tstartsweep;
unsigned long period = 100; // Timeout safety margin
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

  // Safety Timeout
  if (millis() - tstartsweep > period) {
    sweep_active = false;
    return;
  }

  int sample = analogRead(pin_input1);

  // --- STATE MACHINE: CHRONOLOGICAL PEAK DETECTION ---

  // PEAK 0: First Big Peak (Master 1)
  if (counter == 0 && sample > High_threshold1) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold1 && i < arraysize);
    
    unsigned long peak_offset = peakfinder(i, micros() - time_peak);
    t01 = time_peak - start_time + peak_offset;
    counter++; 
  }

  // PEAK 1: First Small Peak (Slave)
  else if (counter == 1 && sample > High_threshold2) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold2 && i < arraysize);
    
    unsigned long peak_offset = peakfinder(i, micros() - time_peak);
    t2 = time_peak - start_time + peak_offset; // Save as SignalPeak
    counter++;
  }

  // PEAK 2: Second Small Peak (SKIP)
  else if (counter == 2 && sample > High_threshold2) {
    // We detect it to move the counter, but we DO NOT save the time.
    int i = 0;
    do {
      sample = analogRead(pin_input1); // Just consume the signal
      i++;
    } while (sample > Low_threshold2 && i < arraysize);
    
    counter++; // Move to look for the final Big Peak
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
    
    unsigned long peak_offset = peakfinder(i, micros() - time_peak);
    t02 = time_peak - start_time + peak_offset;
    counter++;
  }

  // --- Final Calculations and Output ---
  if (counter >= 4) {
    sweep_active = false;
    double alpha2 = (double)(t2 - t01) / (t02 - t01);
    double lock_error = alpha2_ref - alpha2;

    Serial.print("Peak01: "); Serial.print(t01);
    Serial.print(" | SignalPeak: "); Serial.print(t2);
    Serial.print(" | Peak02: "); Serial.print(t02);
    Serial.print(" | Lock Error: "); Serial.println(lock_error, 6);
  }

  prev_trigger_state = trigger_now;
}

// Savitzky-Golay Peak Finder
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  int dsignalarray[arraysize];
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
