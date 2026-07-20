// --- Configuration and Thresholds ---
#define High_threshold1 2950   // Big Peak (Master)
#define Low_threshold1 2850
#define High_threshold2 1400   // Small Peak (Slave)
#define Low_threshold2 1300

#define pin_input1 A8          // Photodiode signal
#define dpin_in 8              // Manual lock switch
#define dpin_out 12            // Trigger input
#define pin_output2 DAC1       // Feedback to Slave Laser
#define arraysize 2000
#define alpha2_ref 0.50

// --- PID / Servo Variables ---
int sign2 = -1;                // Adjust based on laser response
volatile float laser2_K_p = 0.7; 
volatile float laser2_K_i = 1.0;
float laser2_error_signal_current = 0;
float laser2_error_signal_prev = 0;
float laser2_control_signal = 2048; // Start at mid-rail (1.65V)

// --- Global Acquisition Variables ---
unsigned long t01, t02, t2, start_time, time_peak, tstartsweep;
unsigned long period = 100;    // 100ms safety timeout
int signalarray[arraysize];
bool sweep_active = false;
bool prev_trigger_state = false;
int counter = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Serial.println("Complete System: Locking 1st Small Peak to Big Peaks.");
}

void loop() {
  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  // Start Sweep on Trigger Edge
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

  // --- STATE MACHINE ---

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

  // PEAK 1: First Small Peak (Slave - Targeted for Lock)
  else if (counter == 1 && sample > High_threshold2) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold2 && i < arraysize);
    t2 = time_peak - start_time + peakfinder(i, micros() - time_peak);
    counter++;
  }

  // PEAK 2: Second Small Peak (SKIP)
  else if (counter == 2 && sample > High_threshold2) {
    int i = 0;
    do {
      sample = analogRead(pin_input1);
    } while (sample > Low_threshold2 && i < arraysize);
    counter++; // CRITICAL: Increment counter to move to the next Big Peak
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
  }

  // --- CALCULATE ERROR AND APPLY FEEDBACK ---
  if (counter >= 4) {
    sweep_active = false;
    double alpha2 = (double)(t2 - t01) / (t02 - t01);
    laser2_error_signal_current = alpha2_ref - alpha2;

    // Velocity PI Algorithm [1, 2]
    float delta_u = (laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev)) + 
                    (laser2_K_i * laser2_error_signal_current * 0.05); // 0.05 is dt (50ms)
    
    float new_control = laser2_control_signal + (sign2 * delta_u * 4095);

    // Anti-Windup / Rail Protection [2]
    if (new_control >= 0 && new_control <= 4095) {
      laser2_control_signal = new_control;
      analogWrite(pin_output2, (int)laser2_control_signal);
    }

    laser2_error_signal_prev = laser2_error_signal_current;

    // Diagnostics
    Serial.print("Error: "); Serial.print(laser2_error_signal_current, 6);
    Serial.print(" | DAC: "); Serial.println((int)laser2_control_signal);
  }

  prev_trigger_state = trigger_now;
}

// Savitzky-Golay Peak Finder [3, 4]
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
