// --- Configuration and Thresholds ---
#define High_threshold1 1250
#define Low_threshold1 1150
#define High_threshold2 875
#define Low_threshold2 850

#define pin_input1 A4
#define pin_output DAC0
#define dpin_in 8
#define dpin_out 11
#define arraysize 150
#define alpha2_ref 0.50

// --- Global Variables ---
int output = 2800;
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

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(pin_output, OUTPUT);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);

  output = constrain(output, 2400, 3200);
  analogWrite(pin_output, output);

  Serial.println("Sign test: write output, then print peak-system error.");
  Serial.println("Press 's' to start, 'q' to stop");
}

void loop() {
  static unsigned long lastStepMs = 0;
  static bool firstPass = true;

  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's' || cmd == 'S') {
      running = true;
      firstPass = true;
      Serial.println("Test started");
    }
    if (cmd == 'q' || cmd == 'Q') {
      running = false;
      Serial.println("Test stopped");
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
    output += 4;
    if (output > 3200) {
      output = 3200;
    }
    lastStepMs = millis();
    Serial.print("New output = ");
    Serial.println(output);
  }

  int writeValue = constrain(output, 2400, 3200);
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
