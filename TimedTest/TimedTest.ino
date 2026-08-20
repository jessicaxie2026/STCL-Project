// --- Configuration and Thresholds ---
#define REF_START_THRESHOLD 1250
#define REF_END_THRESHOLD 1200

#define pin_input1 A8
#define dpin_in 3       // Input pin for external trigger/lock
#define dpin_out 6      // Digital output for triggering function generator
#define arraysize 2000  // Size of the array for storing peak data

// --- Global Variables ---
unsigned long t01, start_time, end_time, time_peak, tstartsweep;
unsigned long period = 50; // Sweep period limit in ms
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH; // Flag for detecting peak position
bool trigger = false; // Trigger flag for two-pin system
bool sweep_active = false;
bool prev_trigger_state = false;
bool peak_captured = false;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
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
      trigger = true;
      tstartsweep = millis();
      peak_captured = false;
      sweep_active = true;
      Serial.println("Sweep start: trigger fell");
    }
    prev_trigger_state = trigger_now;
    return;
  }

  int sample = analogRead(pin_input1);
  unsigned long timenow = millis();

  if (timenow - tstartsweep > period) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  if (sample > REF_START_THRESHOLD && !peak_captured) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD);

    end_time = micros();
    t01 = (time_peak - start_time) + peakfinder(i, end_time - time_peak);
    peak_captured = true;
    sweep_active = false;
  }

  prev_trigger_state = trigger_now;

  if (!sweep_active && peak_captured) {
    Serial.println(t01);
  }
}

// Savitzky-Golay Filter for peak center detection
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 14) return 0;

  unsigned long dt = duration / number;
  flag = HIGH;

  for (int j = 6; j < (number - 7); j++) {
    // 13-point Savitzky-Golay derivative filter
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
