// --- Configuration and Thresholds ---
// Reference laser (adjusted for lower amplitude peaks)
#define High_threshold1 1100
#define Low_threshold1 1000

// Slave laser (adjusted for lower amplitude peaks)
#define High_threshold2 650
#define Low_threshold2 550

#define pin_input1 A4
#define dpin_in 8       // Manual lock switch
#define dpin_out 11     // Trigger input from Function Gen
#define arraysize 1000  // samples stored per peak (4 KB)
#define ADC_MAX 4095    // 12-bit ADC full scale
#define SLAVE_GATE_US 100000UL  // slave peak must start within 100 ms of reference peak 1
#define alpha2_ref 0.02

// Reasons a peak capture ended
#define END_FELL 0      // signal dropped to the low threshold (normal)
#define END_OVERSHOOT 1 // signal rose above the abort level (reached the big peak)
#define END_FULL 2      // buffer filled

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, tstartsweep;
unsigned long period = 100;   // sweep timeout (ms)
int signalarray[arraysize];
int counter = 0;
bool sweep_active = false;
bool prev_trigger_state = false;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Serial.println("System Ready: Reading Big, Small, (skip) Small, Big sequence.");
}

// Records one peak that starts with `first` (already read).
// Stops when the signal falls to lowThr, rises above abortAbove, or the buffer is full.
// Returns the number of samples stored. tStart = time of first sample,
// duration = time spent reading the following samples, why = END_* reason.
int capturePeak(int first, int lowThr, int abortAbove,
                unsigned long &tStart, unsigned long &duration, int &why, int &lastSample) {
  int sample = first;
  int i = 0;
  tStart = micros();
  why = END_FELL;
  while (true) {
    signalarray[i++] = sample;
    sample = analogRead(pin_input1);
    if (sample <= lowThr)      { why = END_FELL;      break; }
    if (sample > abortAbove)   { why = END_OVERSHOOT; break; }
    if (i >= arraysize)        { why = END_FULL;      break; }
  }
  duration = micros() - tStart;
  lastSample = sample;
  return i;
}

// After a big peak, wait for the tail to fall below the slave low threshold
// so the tail cannot be mistaken for a slave peak. Bounded by the sweep timeout.
void waitForBaseline(int sample) {
  while (sample > Low_threshold2 && (millis() - tstartsweep) <= period) {
    sample = analogRead(pin_input1);
  }
}

// Savitzky-Golay peak finder.
// Returns true and the peak time (us after the first sample) on success,
// false if the window was too short or no derivative zero-crossing was found.
bool peakfinder(int number, unsigned long duration, unsigned long &result) {
  if (number < 13) return false;
  double dt = (double)duration / (double)number;
  long prev_d = 0;

  for (int j = 6; j < (number - 7); j++) {
    long current_d = 6L * signalarray[j+6] + 5L * signalarray[j+5] + 4L * signalarray[j+4] +
                     3L * signalarray[j+3] + 2L * signalarray[j+2] + signalarray[j+1] -
                     signalarray[j-1] - 2L * signalarray[j-2] - 3L * signalarray[j-3] -
                     4L * signalarray[j-4] - 5L * signalarray[j-5] - 6L * signalarray[j-6];

    if (current_d <= 0 && j > 6) {
      long denom = prev_d - current_d;
      double frac = (denom != 0) ? (double)current_d / (double)denom : 0.0;
      double t = ((double)j + frac) * dt;
      if (t < 0.0) t = 0.0;
      result = (unsigned long)(t + 0.5);
      return true;
    }
    prev_d = current_d;
  }
  return false;
}

void loop() {
  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    sweep_active = false;
    counter = 0;
    prev_trigger_state = trigger_now;
    return;
  }

  // Detect Trigger Start (Falling Edge)
  if (!sweep_active) {
    if (!prev_trigger_state && trigger_now) {
      start_time = micros();
      tstartsweep = millis();
      counter = 0;
      t01 = 0;
      t2 = 0;
      t02 = 0;
      sweep_active = true;
    }
    prev_trigger_state = trigger_now;
    return;
  }

  bool sweep_ok = false;   // true only when all three peaks were captured and fitted

  // Safety Timeout: Reset if sweep takes too long
  if (millis() - tstartsweep > period) {
    if (counter == 0) Serial.println("Missing peak: reference peak 1");
    else if (counter == 1) Serial.println("Missing peak: slave peak");
    else Serial.println("Missing peak: reference peak 2");
    sweep_active = false;
    counter = 0;
  } else {
    int sample = analogRead(pin_input1);
    int ref2_sample = -1;            // set when reference peak 2 has started
    unsigned long tS = 0, dur = 0, pf = 0;
    int why = END_FELL, last = 0, n = 0;

    if (counter == 0) {
      // Reference peak 1
      if (sample > High_threshold1) {
        n = capturePeak(sample, Low_threshold1, ADC_MAX, tS, dur, why, last);
        waitForBaseline(last);       // let the tail die out before arming slave detection
        if (peakfinder(n, dur, pf)) {
          t01 = (tS - start_time) + pf;
          counter = 1;
        } else {
          Serial.println("Missing peak: reference peak 1 (peak fit failed)");
          sweep_active = false;
          counter = 0;
        }
      }
    }
    else if (counter == 1) {
      // Slave peak (first of the two)
      if (sample > High_threshold1) {
        // Jumped straight to the big peak: no slave peak in this sweep
        Serial.println("Missing peak: slave peak (reference peak 2 arrived first)");
        sweep_active = false;
        counter = 0;
      }
      else if (sample > High_threshold2) {
        n = capturePeak(sample, Low_threshold2, High_threshold1, tS, dur, why, last);
        unsigned long start_offset = tS - start_time;
        if (why == END_OVERSHOOT) {
          // Climbed into the big peak: that was the rising edge of reference peak 2
          Serial.println("Missing peak: slave peak (reference peak 2 arrived first)");
          sweep_active = false;
          counter = 0;
        }
        else if (start_offset <= t01 || (start_offset - t01) >= SLAVE_GATE_US) {
          Serial.println("Missing peak: slave peak (outside time gate)");
          sweep_active = false;
          counter = 0;
        }
        else if (!peakfinder(n, dur, pf)) {
          Serial.println("Missing peak: slave peak (peak fit failed)");
          sweep_active = false;
          counter = 0;
        }
        else {
          t2 = start_offset + pf;
          counter = 2;
        }
      }
    }
    else {
      // counter 2 or 3: waiting for the second small peak (skipped) and reference peak 2
      if (sample > High_threshold1) {
        ref2_sample = sample;
      }
      else if (counter == 2 && sample > High_threshold2) {
        // Skip the second small peak, but stop if the big peak starts
        while (sample > Low_threshold2 && sample <= High_threshold1 &&
               (millis() - tstartsweep) <= period) {
          sample = analogRead(pin_input1);
        }
        if (sample > High_threshold1) ref2_sample = sample;
        else counter = 3;
      }
    }

    // Reference peak 2
    if (ref2_sample >= 0) {
      n = capturePeak(ref2_sample, Low_threshold1, ADC_MAX, tS, dur, why, last);
      if (peakfinder(n, dur, pf)) {
        t02 = (tS - start_time) + pf;
        sweep_ok = true;
      } else {
        Serial.println("Missing peak: reference peak 2 (peak fit failed)");
      }
      sweep_active = false;
      counter = 0;
    }
  }

  // --- VALIDATION AND OUTPUT ---
  if (!sweep_active) {
    bool valid = sweep_ok && (t2 > t01) && (t2 < t02);

    if (sweep_ok && !valid) {
      Serial.println("Missing peak: invalid sequence");
    }

    if (valid) {
      double alpha2 = (double)(t2 - t01) / (double)(t02 - t01);
      double lock_error = alpha2_ref - alpha2;

      Serial.print(t01); Serial.print(", ");
      Serial.print(t2); Serial.print(", ");
      Serial.print(t02); Serial.print(", Lock Error: ");
      Serial.println(lock_error, 6);
    }
  }

  prev_trigger_state = trigger_now;
}
