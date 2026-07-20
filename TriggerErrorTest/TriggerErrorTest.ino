// --- Configuration and Thresholds ---
#define PEAK_START_THRES 1600   // Safely clears the 1504 baseline/shoulder bump
#define PEAK_END_THRES 1500     // Clean reset threshold back to baseline

#define pin_input1 A8
#define pin_input2 A8
#define dpin_in 8    // Pin for external trigger (manual switch)
#define dpin_out 12  // Digital input from function generator sync/trigger
#define arraysize 2000
#define alpha2_ref 0.50

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep;

// INCREASED TO 55ms: Gives a safety margin for a 20Hz (50ms) ramp
unsigned long period = 55; 

int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH;
double error2, totalT, alpha2;
bool sweep_active = false;
bool prev_trigger_state = false;
int counter = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Serial.println("Setup complete. Waiting for manual switch (Pin 8 to GND)...");
}

// Savitzky-Golay Filter for peak center detection
unsigned long peakfinder(int number, unsigned long duration) {
  unsigned long dt = duration / number;
  flag = HIGH;

  for (int j = 6; j < (number - 7); j++) {
    dsignalarray[j] = (6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] +
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] -
                       dsignalarray[j - 1] - 2 * dsignalarray[j - 2] - 3 * dsignalarray[j - 3] -
                       4 * dsignalarray[j - 4] - 5 * dsignalarray[j - 5] - 6 * dsignalarray[j - 6]);

    if (dsignalarray[j] <= 0 && flag) {
      flag = LOW;
      return (j + (double)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;
    }
  }
  return 0;
}

void loop() {
  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  if (!sweep_active) {
    // Detect trigger edge
    if (!prev_trigger_state && trigger_now) {
      Serial.println(" -> starting sweep");
      start_time = micros();
      tstartsweep = millis();
      counter = 0;
      sweep_active = true;
    }
    prev_trigger_state = trigger_now;
    return;
  }

  int sample = analogRead(pin_input1);

  // Safeguard timeout with diagnostics
  if (millis() - tstartsweep > period) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    
    Serial.print(" Timeout! Stuck at Peak stage: ");
    if (counter == 0) Serial.println("0 (Waiting for 1st Big Reference Peak)");
    else if (counter == 1) Serial.println("1 (Waiting for Target Slave Peak)");
    else if (counter == 2) Serial.println("2 (Waiting for 2nd Slave Peak to Ignore)");
    else if (counter == 3) Serial.println("3 (Waiting for 2nd Big Reference Peak)");
    return;
  }

  // --- UNIVERSAL CHRONOLOGICAL PEAK DETECTION ---
  if (sample > PEAK_START_THRES) {
    time_peak = micros();
    int i = 0;
    
    // Read the current peak into the buffer for the Savitzky-Golay filter
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > PEAK_END_THRES && i < arraysize);

    end_time = micros();

    // 1. First Big Reference Peak
    if (counter == 0) {
      t01 = time_peak - start_time + peakfinder(i, end_time - time_peak);
      counter = 1; // Move to look for the slave peak
    } 
    // 2. Target Slave Peak (The one we WANT to measure)
    else if (counter == 1) {
      t2 = time_peak - start_time + peakfinder(i, end_time - time_peak);
      counter = 2; // Move to the second slave peak (to ignore)
    } 
    // 3. Second Slave Peak (We read it to clear it, but discard the time measurement)
    else if (counter == 2) {
      // The peak has already been completely read past by the do-while loop above.
      counter = 3; // Move to look for the final reference peak
    } 
    // 4. Second Big Reference Peak
    else if (counter == 3) {
      t02 = time_peak - start_time + peakfinder(i, end_time - time_peak);
      counter = 4; // Sweep successfully complete!
    }
  }

  // Wrap up sweep calculations if all 4 steps finished successfully
  if (counter >= 4) {
    sweep_active = false;
    error2 = (double)t2 - t01;
    totalT = (double)t02 - t01;
    alpha2 = error2 / totalT;
    double lock_error = alpha2_ref - alpha2;

    Serial.print("Peak01 (Ref): "); Serial.print(t01);
    Serial.print(" | SignalPeak (Slave): "); Serial.print(t2);
    Serial.print(" | Peak02 (Ref): "); Serial.print(t02);
    Serial.print(" | Lock Error: "); Serial.println(lock_error, 6);
  }

  prev_trigger_state = trigger_now;
}
