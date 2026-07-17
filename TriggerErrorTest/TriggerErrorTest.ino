// --- Configuration and Thresholds ---
#define REF_START_THRESHOLD 2900
#define REF_END_THRESHOLD 2800
#define SLAVE_START_THRESHOLD 1400
#define SLAVE_END_THRESHOLD 1300

#define pin_input1 A8
#define pin_input2 A8
#define dpin_in 8    // Pin for external trigger
#define dpin_out 10      // Digital output for triggering function generator
#define arraysize 2000
#define alpha2_ref 0.50

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep;
unsigned long period = 50; // Sweep period limit in ms
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH;
double error2, totalT, alpha2;
bool sweep_active = false;
bool prev_trigger_state = false;
bool indicator2 = false;
int counter = 0;

// Trigger state will be polled from `dpin_in`

// Using TriggerCheck-style polling: dpin_in acts as lock (INPUT_PULLUP)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Setup trigger pin for polling
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Serial.println("Setup complete");
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

void loop() {
  Serial.println("Loop running");
  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  if (!sweep_active) {
    if (!prev_trigger_state && trigger_now) {
      Serial.println(" -> starting sweep");
      start_time = micros();
      tstartsweep = millis();
      counter = 0;
      indicator2 = false;
      sweep_active = true;
    }
    prev_trigger_state = trigger_now;
    return;
  }

  bool timed_out = false;
  int sample = analogRead(pin_input1);

  if (millis() - tstartsweep > period) {
    timed_out = true;
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  if (counter == 0 && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    end_time = micros();
    unsigned long p_time = time_peak - start_time + peakfinder(i, end_time - time_peak);
    t01 = p_time;
    counter = 1;
  }
  else if (counter == 1 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD && !indicator2) {
    indicator2 = true;
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > SLAVE_END_THRESHOLD && i < arraysize);

    end_time = micros();
    t2 = time_peak - start_time + peakfinder(i, end_time - time_peak);
    counter = 2;
  }
  else if (counter == 2 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
    do {
      sample = analogRead(pin_input1);
    } while (sample > SLAVE_END_THRESHOLD && sample > 0);
  }
  else if (counter == 2 && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    end_time = micros();
    t02 = time_peak - start_time + peakfinder(i, end_time - time_peak);
    counter = 3;
    sweep_active = false;
  }

  if (counter >= 3) sweep_active = false;
  prev_trigger_state = trigger_now;

  if (!sweep_active) {
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
      if (timed_out) {
        Serial.println(" Timeout - failed to capture all 3 peaks. Resetting outputs.");
      } else {
        Serial.println(" Failed to capture all 3 peaks.");
      }
    }
  }
}
