// --- Configuration and Thresholds from Source [6] ---
#define High_threshold1 1250    // Reference laser (Big Peaks)
#define Low_threshold1 1200
#define High_threshold2 900    // Slave laser (Small Peaks)
#define Low_threshold2 885

#define pin_input1 A8          // Photodiode 1 (D2 and 935nm)
#define pin_input2 A8          // Photodiode 2 (795nm)
#define arraysize 2000
#define dpin_in 8              // Manual switch
#define dpin_out 12            // Trigger input
#define alpha2_ref 0.50
#define pin_output2 DAC1

// --- Global Variables and PID Parameters from Source [7, 8] ---
unsigned long t01, t02, t1, t2, start_time, end_time, tstartsweep, time_peak, timenow;
unsigned long period = 55; 
int signalarray[arraysize];
int counter, Range;
double value1, value2;
float offset = 3733.0;
float error = 0.0;
float alpha2, error2, totalT;
bool sweep_active = false;
bool prev_trigger_state = false;

// PID variables from your original code [8]
int sign2 = -1;
volatile float laser2_K_i = 1;
volatile float laser2_K_p = 0.7;
float laser2_error_signal_current;
float laser2_error_signal_prev;
float laser2_control_signal = 0;

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Range = (pow(2, 12) - 1) - 200; // Original Range calculation [8]
  Serial.println("Setup complete");
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
    if (!prev_trigger_state && trigger_now) {
      start_time = micros();
      tstartsweep = millis();
      counter = 0;
      sweep_active = true;
    }
    prev_trigger_state = trigger_now;
    return;
  }

  if (millis() - tstartsweep > period) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  int sample = analogRead(pin_input1);

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
    }
  }

  else if (counter == 2 && sample > High_threshold2) {
    int i = 0;
    do {
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold2 && i < arraysize);
    counter++;
  }

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

  if (counter >= 4) {
    sweep_active = false;

    if (t2 > t01 && t2 < t02) {
      error2 = (double)t2 - t01;
      totalT = (double)t02 - t01;
      alpha2 = error2 / totalT;
      laser2_error_signal_current = alpha2_ref - alpha2;

      float delta_laser2 = (laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev)) +
                           (laser2_K_i * laser2_error_signal_current);
      laser2_control_signal += (sign2 * delta_laser2);

      if (laser2_control_signal >= 0 && laser2_control_signal <= Range) {
        error = offset + (Range / 2.0) * laser2_control_signal;
        if (error > DAC_MAX_COUNTS) error = DAC_MAX_COUNTS;
        if (error < DAC_MIN_COUNTS) error = DAC_MIN_COUNTS;
        analogWrite(pin_output2, (int)error);
      }

      laser2_error_signal_prev = laser2_error_signal_current;

      Serial.print(t01); Serial.print(", ");
      Serial.print(t2); Serial.print(", ");
      Serial.print(t02); Serial.print(", Lock Error: ");
      Serial.println(laser2_error_signal_current, 6);
    } else {
      counter = 0;
      Serial.println("Glitchy sequence");
    }
  }

  prev_trigger_state = trigger_now;
}

// Savitzky-Golay Peak Finder from Source [9]
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  int dsignalarray[arraysize];
  int prev_d = 0;

  for (int j = 6; j < (number - 7); j++) {
    int current_d = int((6 * signalarray[j+6] + 5 * signalarray[j+5] + 4 * signalarray[j+4] + 
                         3 * signalarray[j+3] + 2 * signalarray[j+2] + signalarray[j+1] - 
                         signalarray[j-1] - 2 * signalarray[j-2] - 3 * signalarray[j-3] - 
                         4 * signalarray[j-4] - 5 * signalarray[j-5] - 6 * signalarray[j-6]));
    
    if (current_d <= 0 && j > 6) {
      return (unsigned long)((j + (double)current_d / (prev_d - current_d)) * dt);
    }
    prev_d = current_d;
  }
  return 0;
}
