// --- Configuration and Thresholds ---
#define High_threshold1 1100
#define Low_threshold1 1000
#define High_threshold2 650
#define Low_threshold2 630

#define pin_input1 A4
#define pin_output DAC0
#define dpin_in 8
#define dpin_out 11
#define arraysize 150


// --- Global Variables ---
int high = 3000;
int low = 1700;
double alpha2_ref = 0.02;   // now adjustable at runtime
int output = 2661;
int increment = 3;
int signalarray[arraysize];
bool running = false;
unsigned long t01 = 0;
unsigned long t02 = 0;
unsigned long t2 = 0;
unsigned long start_time = 0;
unsigned long time_peak = 0;
unsigned long tstartsweep = 0;
unsigned long period = 100;
bool sweep_active = false;
bool prev_trigger_state = false;
int counter = 0;

void handleCommand(char *cmd) {
  char c = tolower(cmd[0]);
  char *arg = cmd + 1;

  if (c == 's') {
    running = true;
    Serial.println("Test started");
  }
  else if (c == 'q') {
    running = false;
    Serial.println("Test stopped");
  }
  else if (c == 'o') {                 // o2300 -> set output absolute
    output = constrain(atoi(arg), low, high);
    analogWrite(pin_output, output);
    Serial.print("Output = "); Serial.println(output);
  }
  else if (c == '+' || c == '-') {     // +10 / -25 -> shift output by N counts
    output = constrain(output + atoi(cmd), low, high);
    analogWrite(pin_output, output);
    Serial.print("Output = "); Serial.println(output);
  }
  else if (c == 'a') {                 // a0.52 -> set ref;  a+0.01 / a-0.01 -> shift ref
    if (arg[0] == '+' || arg[0] == '-') alpha2_ref += atof(arg);
    else alpha2_ref = atof(arg);
    Serial.print("alpha2_ref = "); Serial.println(alpha2_ref, 6);
  }
  else if (c == 'i') {                 // i0 -> stop auto-ramp, i3 -> ramp 3 counts per 5 s
    increment = atoi(arg);
    Serial.print("Auto increment = "); Serial.println(increment);
  }
  else if (c == 'p') {                 // print status
    Serial.print("output = "); Serial.print(output);
    Serial.print("  alpha2_ref = "); Serial.print(alpha2_ref, 6);
    Serial.print("  increment = "); Serial.println(increment);
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(pin_output, OUTPUT);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);

  output = constrain(output, low, high);
  analogWrite(pin_output, output);

  Serial.println("Commands: s start, q stop, +N/-N shift output, oN set output, a0.5 set ref, a+0.01 shift ref, iN auto-step, p status");
}

void loop() {
  static unsigned long lastStepMs = 0;
  static bool firstPass = true;

    static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) {
        buf[n] = 0;
        char c0 = tolower(buf[0]);
        if (c0 == 's') firstPass = true;   // keep the original restart behavior
        handleCommand(buf);
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }

  if (!running) {
    return;
  }

  if (firstPass) {
    firstPass = false;
    lastStepMs = millis();
  }

  if (millis() - lastStepMs >= 5000UL) {
    output += increment;
    if (output > high) {
      output = high;
    }
    if (output < low){
      output = low;
    }
    lastStepMs = millis();
    Serial.print("New output = ");
    Serial.println(output);
  }

  int writeValue = constrain(output, low, high);
  analogWrite(pin_output, writeValue);

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

  int sample = 0;
  double error = 9999.0;
  if (millis() - tstartsweep > period) {
    if (counter == 0) Serial.println("Missing peak: reference peak 1");
    else if (counter == 1) Serial.println("Missing peak: slave peak");
    else if (counter == 3) Serial.println("Missing peak: reference peak 2");
    else Serial.println("Missing peak: unknown peak");
    sweep_active = false;
    counter = 0;
  } else {
    sample = analogRead(pin_input1);

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
      } else {
        Serial.println("Missing peak: slave peak");
        sweep_active = false;
        counter = 0;
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
      sweep_active = false;
      counter++;
    }
  }

  if (!sweep_active && t01 != 0 && t2 != 0 && t02 != 0 && t2 > t01 && t2 < t02) {
    double alpha2 = (double)(t2 - t01) / (double)(t02 - t01);
    error = alpha2_ref - alpha2;
  }

  if (!sweep_active) {
    Serial.print("output = ");
    Serial.print(writeValue);
    Serial.print("  alpha2 = ");
    if (t02 > t01) {
      Serial.print((double)(t2 - t01) / (double)(t02 - t01), 6);
    } else {
      Serial.print("N/A");
    }
    Serial.print("  Lock Error: ");
    Serial.println(error, 6);
  }

  prev_trigger_state = trigger_now;
}

unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  int prev_d = 0;

  for (int j = 6; j < (number - 7); j++) {
    int current_d = int(6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] +
                         3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] -
                         signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] -
                         4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]);
    if (current_d <= 0 && prev_d > 0) {
      return (unsigned long)((j + (double)current_d / (prev_d - current_d)) * dt);
    }
    prev_d = current_d;
  }
  return 0;
}
