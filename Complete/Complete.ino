// --- Configuration and Thresholds ---
#define High_threshold1 1100    // Reference laser (Big Peaks)
#define Low_threshold1 1000
#define High_threshold2 650     // Slave laser (Small Peaks)
#define Low_threshold2 630

#define pin_input1 A4
#define arraysize 1000          // samples stored per peak (4 KB)
#define ADC_MAX 4095            // 12-bit ADC full scale
#define SLAVE_GATE_US 100000UL  // slave peak must start within 100 ms of reference peak 1
#define dpin_in 8               // Manual switch
#define dpin_out 11             // Trigger input
#define pin_output2 DAC0

// Reasons a peak capture ended
#define END_FELL 0              // signal dropped to the low threshold (normal)
#define END_OVERSHOOT 1         // signal rose above the abort level (reached the big peak)
#define END_FULL 2              // buffer filled

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, tstartsweep, time_peak;
unsigned long period = 100;     // sweep timeout (ms)
int signalarray[arraysize];
int counter;
int Range = 600;                // control swing: output = offset +/- Range/2
float offset = 2615.0;          // midpoint of the output (DAC counts)
int dac_output = 2000;          // last value written to the DAC
bool sweep_active = false;
bool prev_trigger_state = false;
bool running = false;           // start stopped; send 's' to run
bool pid_active = true;         // false = PID stopped, DAC holds, error still reported

// Runtime-adjustable parameters
double alpha2_ref = 0.02;
float laser2_K_p = 3;
float laser2_K_i = 4;
int sign2 = 1;

const float DAC_MIN_COUNTS = 1700.0f;
const float DAC_MAX_COUNTS = 3000.0f;

float laser2_error_signal_current = 0;
float laser2_error_signal_prev = 0;
float laser2_control_signal = 300;   // Range / 2 (set again in setup)

static inline float clampDACValue(float value) {
  if (value > DAC_MAX_COUNTS) return DAC_MAX_COUNTS;
  if (value < DAC_MIN_COUNTS) return DAC_MIN_COUNTS;
  return value;
}

// Make the controller state match the current DAC value (no jump when the PID starts or resumes)
void syncControlToDAC() {
  float c = (float)dac_output - offset + (Range / 2.0f);
  if (c > Range) c = (float)Range;
  if (c < 0.0f) c = 0.0f;
  laser2_control_signal = c;
  laser2_error_signal_prev = 0;
}

// Per-sweep report: output, alpha2, lock error, then peak times
void reportSweep(bool valid, double alpha, double lockError) {
  Serial.print("output = ");
  Serial.print(dac_output);
  Serial.print("  alpha2 = ");
  if (valid) {
    Serial.print(alpha, 6);
  } else {
    Serial.print("N/A");
  }
  Serial.print("  Lock Error: ");
  Serial.println(lockError, 6);

  Serial.print(t01); Serial.print(", ");
  Serial.print(t2); Serial.print(", ");
  Serial.print(t02); Serial.print(", Lock Error: ");
  Serial.println(lockError, 6);
}

// Parses "+0.1" / "-0.1" as a shift, anything else as an absolute value
double applyValue(double current, const char *arg) {
  if (arg[0] == '+' || arg[0] == '-') return current + atof(arg);
  return atof(arg);
}

void handleCommand(char *cmd) {
  char c = tolower(cmd[0]);

  if (c == 's') {                       // start (PID on)
    syncControlToDAC();
    sweep_active = false;
    counter = 0;
    pid_active = true;
    running = true;
    Serial.println("Lock started");
  }
  else if (c == 'q') {                  // stop everything (DAC holds last value, no error output)
    running = false;
    sweep_active = false;
    counter = 0;
    Serial.println("Lock stopped");
  }
  else if (c == 'h') {                  // stop the PID only: keep measuring, DAC holds
    pid_active = false;
    Serial.print("PID stopped, DAC holding at "); Serial.println(dac_output);
  }
  else if (c == 'g') {                  // resume the PID from the current DAC value
    syncControlToDAC();
    pid_active = true;
    Serial.print("PID resumed from DAC = "); Serial.println(dac_output);
  }
  else if (c == 'o') {                  // o2400 -> set DAC; o+10 / o-10 -> shift DAC (PID must be stopped)
    if (running && pid_active) {
      Serial.println("PID is running. Send h to stop it first.");
    } else if (cmd[1] == 0) {
      Serial.println("Use oN (absolute) or o+N / o-N (shift)");
    } else {
      double v = applyValue((double)dac_output, cmd + 1);
      dac_output = (int)clampDACValue((float)v);
      analogWrite(pin_output2, dac_output);
      Serial.print("DAC output = "); Serial.println(dac_output);
    }
  }
  else if (c == 'k') {                  // kp0.7 / ki1.0 / kp+0.1 / ki-0.1
    char which = tolower(cmd[1]);
    char *arg = cmd + 2;
    if (which == 'p') {
      laser2_K_p = applyValue(laser2_K_p, arg);
      Serial.print("Kp = "); Serial.println(laser2_K_p, 6);
    } else if (which == 'i') {
      laser2_K_i = applyValue(laser2_K_i, arg);
      Serial.print("Ki = "); Serial.println(laser2_K_i, 6);
    } else {
      Serial.println("Use kpX or kiX");
    }
  }
  else if (c == 'a') {                  // a0.52 / a+0.01 / a-0.01
    alpha2_ref = applyValue(alpha2_ref, cmd + 1);
    Serial.print("alpha2_ref = "); Serial.println(alpha2_ref, 6);
  }
  else if (c == 'd') {                  // d1 / d-1 -> set sign2; d -> flip it
    char *arg = cmd + 1;
    if (arg[0] == 0) {
      sign2 = -sign2;
    } else {
      int v = atoi(arg);
      if (v > 0) sign2 = 1;
      else if (v < 0) sign2 = -1;
      else { Serial.println("Use d1, d-1, or d to flip"); return; }
    }
    Serial.print("sign2 = "); Serial.println(sign2);
  }
  else if (c == 'f') {                  // f2000 -> DAC offset (counts)
    offset = applyValue(offset, cmd + 1);
    Serial.print("offset = "); Serial.println(offset, 1);
  }
  else if (c == 'r') {                  // reset controller to the midpoint
    laser2_control_signal = Range / 2.0;
    laser2_error_signal_prev = 0;
    dac_output = (int)clampDACValue(offset);
    analogWrite(pin_output2, dac_output);
    Serial.println("Controller reset to midpoint");
  }
  else if (c == 'p') {                  // print status
    Serial.print("running = "); Serial.print(running);
    Serial.print("  pid_active = "); Serial.print(pid_active);
    Serial.print("  output = "); Serial.print(dac_output);
    Serial.print("  alpha2_ref = "); Serial.print(alpha2_ref, 6);
    Serial.print("  Kp = "); Serial.print(laser2_K_p, 6);
    Serial.print("  Ki = "); Serial.print(laser2_K_i, 6);
    Serial.print("  sign2 = "); Serial.print(sign2);
    Serial.print("  offset = "); Serial.print(offset, 1);
    Serial.print("  control = "); Serial.println(laser2_control_signal, 2);
  }
  else {
    Serial.println("Unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);

  laser2_control_signal = Range / 2.0;           // start at the midpoint
  dac_output = (int)clampDACValue(offset);       // remove these two lines if you
  analogWrite(pin_output2, dac_output);          // don't want DAC output at power-up

  Serial.println("Commands: s start, q stop all, h stop PID (keep error output), g resume PID, oN set DAC (PID stopped), kpX Kp, kiX Ki, aX alpha ref, d1/d-1/d sign, fX offset, r reset, p status");
  Serial.println("X may be absolute (kp0.7, o2400) or a shift (kp+0.1, o+10)");
}

void readSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) {
        buf[n] = 0;
        handleCommand(buf);
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
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
  readSerial();

  if (!running) return;

  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    sweep_active = false;
    counter = 0;
    prev_trigger_state = trigger_now;
    return;
  }

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

  bool sweep_ok = false;   // true only when all three peaks were captured

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
        if (!peakfinder(n, dur, pf)) pf = 0;   // fall back to threshold-crossing time
        t01 = (tS - start_time) + pf;
        counter = 1;
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
        else {
          if (!peakfinder(n, dur, pf)) pf = 0;   // fall back to threshold-crossing time
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
      if (!peakfinder(n, dur, pf)) pf = 0;   // fall back to threshold-crossing time
      t02 = (tS - start_time) + pf;
      sweep_ok = true;
      sweep_active = false;
      counter = 0;
    }
  }

  // End of sweep (complete or failed): compute error, update controller (if PID active), report
  if (!sweep_active) {
    double lockError = 9999.0;
    double alpha = 0.0;
    bool valid = sweep_ok && (t2 > t01) && (t2 < t02);

    if (sweep_ok && !valid) Serial.println("Missing peak: invalid sequence");

    if (valid) {
      alpha = (double)(t2 - t01) / (double)(t02 - t01);
      lockError = alpha2_ref - alpha;

      if (pid_active) {
        laser2_error_signal_current = lockError;

        float delta_laser2 = (laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev)) +
                             (laser2_K_i * laser2_error_signal_current);
        laser2_control_signal += (sign2 * delta_laser2);
        if (laser2_control_signal > Range) laser2_control_signal = (float)Range;
        if (laser2_control_signal < 0.0f) laser2_control_signal = 0.0f;

        float dac_target = offset + laser2_control_signal - (Range / 2.0);
        dac_output = (int)clampDACValue(dac_target);
        analogWrite(pin_output2, dac_output);

        laser2_error_signal_prev = laser2_error_signal_current;
      }
    }

    reportSweep(valid, alpha, lockError);
  }

  prev_trigger_state = trigger_now;
}