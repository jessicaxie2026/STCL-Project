// --- Configuration and Thresholds ---
#define High_threshold1 1100    // Reference laser (Big Peaks)
#define Low_threshold1 1000
#define High_threshold2 650     // Slave laser (Small Peaks)
#define Low_threshold2 550

#define pin_input1 A4
#define arraysize 2000
#define dpin_in 8               // Manual switch
#define dpin_out 11             // Trigger input
#define pin_output2 DAC0

// --- Global Variables ---
unsigned long t01, t02, t2, start_time, tstartsweep, time_peak;
unsigned long period = 100;
int signalarray[arraysize];
int counter;
int Range = 600;                // control swing: output = offset +/- Range/2
float offset = 2000.0;          // midpoint of the output (DAC counts)
float alpha2, error2, totalT;
float error = 0.0;
int dac_output = 2000;          // last value written to the DAC
bool sweep_active = false;
bool prev_trigger_state = false;
bool running = false;           // start stopped; send 's' to run

// Runtime-adjustable parameters
double alpha2_ref = 0.50;
float laser2_K_p = 0.7;
float laser2_K_i = 1.0;
int sign2 = -1;

const float DAC_MIN_COUNTS = 1700.0f;
const float DAC_MAX_COUNTS = 2300.0f;

float laser2_error_signal_current = 0;
float laser2_error_signal_prev = 0;
float laser2_control_signal = 300;   // Range / 2 (set again in setup)

static inline float clampDACValue(float value) {
  if (value > DAC_MAX_COUNTS) return DAC_MAX_COUNTS;
  if (value < DAC_MIN_COUNTS) return DAC_MIN_COUNTS;
  return value;
}

void resetSweep() {
  sweep_active = false;
  counter = 0;
  t01 = 0;
  t2 = 0;
  t02 = 0;
}

// Same per-sweep report as Code 1: output, alpha2, lock error
void reportSweep(double lockError) {
  Serial.print("output = ");
  Serial.print(dac_output);
  Serial.print("  alpha2 = ");
  if (t02 > t01 && t2 > t01) {
    Serial.print((double)(t2 - t01) / (double)(t02 - t01), 6);
  } else {
    Serial.print("N/A");
  }
  Serial.print("  Lock Error: ");
  Serial.println(lockError, 6);
}

// Failed sweep: report with error 9999 (like Code 1), then reset
void failSweep() {
  reportSweep(9999.0);
  resetSweep();
}

// Parses "+0.1" / "-0.1" as a shift, anything else as an absolute value
double applyValue(double current, const char *arg) {
  if (arg[0] == '+' || arg[0] == '-') return current + atof(arg);
  return atof(arg);
}

void handleCommand(char *cmd) {
  char c = tolower(cmd[0]);

  if (c == 's') {                       // start
    resetSweep();
    laser2_error_signal_prev = 0;
    running = true;
    Serial.println("Lock started");
  }
  else if (c == 'q') {                  // stop (DAC holds last value)
    running = false;
    resetSweep();
    Serial.println("Lock stopped");
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
    Serial.print("  output = "); Serial.print(dac_output);
    Serial.print("  alpha2_ref = "); Serial.print(alpha2_ref, 6);
    Serial.print("  Kp = "); Serial.print(laser2_K_p, 6);
    Serial.print("  Ki = "); Serial.print(laser2_K_i, 6);
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

  Serial.println("Commands: s start, q stop, kpX Kp, kiX Ki, aX alpha ref, fX offset, r reset, p status");
  Serial.println("X may be absolute (kp0.7) or a shift (kp+0.1)");
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

void loop() {
  readSerial();

  if (!running) return;

  bool manual_now = (digitalRead(dpin_in) == LOW);
  bool trigger_now = (digitalRead(dpin_out) == LOW);

  if (!manual_now) {
    resetSweep();
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
    if (counter == 0) Serial.println("Missing peak: reference peak 1");
    else if (counter == 1) Serial.println("Missing peak: slave peak");
    else if (counter == 3) Serial.println("Missing peak: reference peak 2");
    else Serial.println("Missing peak: unknown peak");
    failSweep();
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
    unsigned long peak_offset = peakfinder(i, micros() - time_peak);
    if (peak_offset == 0) {
      Serial.println("Invalid peak: reference peak 1");
      failSweep();
    } else {
      t01 = time_peak - start_time + peak_offset;
      counter++;
    }
  }

  else if (counter == 1 && sample > High_threshold2 && sample < High_threshold1) {
    time_peak = micros();
    unsigned long current_offset = time_peak - start_time;

    if (current_offset > t01 && (current_offset - t01) < 10000) {
      int i = 0;
      do {
        if (i < arraysize) signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > Low_threshold2 && i < arraysize);
      unsigned long peak_offset = peakfinder(i, micros() - time_peak);
      if (peak_offset == 0) {
        Serial.println("Invalid peak: slave peak");
        failSweep();
      } else {
        t2 = current_offset + peak_offset;
        counter++;
      }
    } else {
      Serial.println("Missing peak: slave peak");
      failSweep();
    }
  }

  else if (counter == 1 && sample > High_threshold1) {
    Serial.println("Invalid sequence: reference peak before second slave peak");
    failSweep();
  }

  else if (counter == 2 && sample > High_threshold2 && sample < High_threshold1) {
    int i = 0;
    do {
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold2 && i < arraysize);
    counter++;
  }

  else if (counter == 2 && sample > High_threshold1) {
    Serial.println("Invalid sequence: reference peak before second slave peak");
    failSweep();
  }

  else if (counter == 3 && sample > High_threshold1) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > Low_threshold1 && i < arraysize);
    unsigned long peak_offset = peakfinder(i, micros() - time_peak);
    if (peak_offset == 0) {
      Serial.println("Invalid peak: reference peak 2");
      failSweep();
    } else {
      t02 = time_peak - start_time + peak_offset;
      counter++;
    }
  }

  if (counter >= 4) {
    sweep_active = false;

    if (t2 > t01 && t2 < t02 && t02 > t01) {
      error2 = (double)t2 - t01;
      totalT = (double)t02 - t01;
      alpha2 = error2 / totalT;
      laser2_error_signal_current = alpha2_ref - alpha2;

      float delta_laser2 = (laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev)) +
                           (laser2_K_i * laser2_error_signal_current);
      laser2_control_signal += (sign2 * delta_laser2);
      if (laser2_control_signal > Range) laser2_control_signal = (float)Range;
      if (laser2_control_signal < 0.0f) laser2_control_signal = 0.0f;

      error = offset + laser2_control_signal - (Range / 2.0);
      dac_output = (int)clampDACValue(error);
      analogWrite(pin_output2, dac_output);

      laser2_error_signal_prev = laser2_error_signal_current;

      reportSweep(laser2_error_signal_current);
    } else {
      Serial.println("Glitchy sequence");
      reportSweep(9999.0);
    }
    counter = 0;
    t01 = 0;
    t2 = 0;
    t02 = 0;
  }

  prev_trigger_state = trigger_now;
}

// Savitzky-Golay Peak Finder
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  int prev_d = 0;

  for (int j = 6; j < (number - 7); j++) {
    int current_d = int(6 * signalarray[j+6] + 5 * signalarray[j+5] + 4 * signalarray[j+4] +
                        3 * signalarray[j+3] + 2 * signalarray[j+2] + signalarray[j+1] -
                        signalarray[j-1] - 2 * signalarray[j-2] - 3 * signalarray[j-3] -
                        4 * signalarray[j-4] - 5 * signalarray[j-5] - 6 * signalarray[j-6]);

    if (current_d <= 0 && prev_d > 0) {
      return (unsigned long)((j + (double)current_d / (prev_d - current_d)) * dt);
    }
    prev_d = current_d;
  }
  return 0;
}
