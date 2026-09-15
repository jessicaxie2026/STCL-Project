#define pin_input1 A8
#define pin_output DAC0

#define REF_START_THRESHOLD 1250
#define REF_END_THRESHOLD 950
#define SLAVE_START_THRESHOLD 800
#define SLAVE_END_THRESHOLD 750
#define alpha2_ref 0.50
#define arraysize 2000

int output = 2800;
int signalarray[arraysize];

bool sweep_active = false;
bool prev_trigger_state = false;
int counter = 0;
int sweep_count = 0;

unsigned long t01 = 0;
unsigned long t2 = 0;
unsigned long t02 = 0;
unsigned long start_time = 0;
unsigned long time_peak = 0;
unsigned long tstartsweep = 0;
unsigned long period = 100;
#define dpin_in 8
#define dpin_out 11

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(pin_output, OUTPUT);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);

  output = constrain(output, 2400, 3200);
  analogWrite(pin_output, output);

  Serial.println("System Ready: waiting for manual lock switch and trigger.");
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

      int writeValue = constrain(output, 2400, 3200);
      writeValue = min(writeValue, 3200);
      analogWrite(pin_output, writeValue);

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
    sweep_active = false;
    counter = 0;
    return;
  }

  int sample = analogRead(pin_input1);

  if (counter == 0 && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    t01 = time_peak - start_time + peakfinder(i, micros() - time_peak);
    counter++;
  }

  else if (counter == 1 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
    time_peak = micros();
    unsigned long current_offset = time_peak - start_time;

    if (current_offset > t01 && (current_offset - t01) < 10000) {
      int i = 0;
      do {
        if (i < arraysize) signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > SLAVE_END_THRESHOLD && i < arraysize);

      t2 = current_offset + peakfinder(i, micros() - time_peak);
      counter++;
    } else {
      Serial.println("Missing peak: slave peak");
      sweep_active = false;
      counter = 0;
    }
  }

  else if (counter == 1 && sample > REF_START_THRESHOLD) {
    Serial.println("Invalid sequence: reference peak before second slave peak");
    sweep_active = false;
    counter = 0;
  }

  else if (counter == 2 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
    int i = 0;
    do {
      sample = analogRead(pin_input1);
      i++;
    } while (sample > SLAVE_END_THRESHOLD && i < arraysize);
    counter++;
  }

  else if (counter == 2 && sample > REF_START_THRESHOLD) {
    Serial.println("Invalid sequence: reference peak before second slave peak");
    sweep_active = false;
    counter = 0;
  }

  else if (counter == 3 && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    t02 = time_peak - start_time + peakfinder(i, micros() - time_peak);
    counter++;
  }

  if (counter >= 4) {
    sweep_active = false;

    int writeValue = constrain(output, 2400, 3200);
    writeValue = min(writeValue, 3200);
    double error = 9999.0;

    if (t2 > t01 && t2 < t02 && t02 > t01) {
      double alpha2 = (double)(t2 - t01) / (double)(t02 - t01);
      error = alpha2_ref - alpha2;
    }

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

    sweep_count++;
    if (sweep_count >= 100) {
      output = constrain(output + 11, 2400, 3200);
      analogWrite(pin_output, output);
      sweep_count = 0;
    }

    counter = 0;
    t01 = 0;
    t2 = 0;
    t02 = 0;
  }

  prev_trigger_state = trigger_now;
}

unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  int prev_d = 0;
  int dsignalarray[arraysize];

  for (int j = 6; j < (number - 7); j++) {
    dsignalarray[j] = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] +
                          3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] -
                          signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] -
                          4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]));

    if (dsignalarray[j] <= 0 && prev_d > 0) {
      return (unsigned long)((j + (double)dsignalarray[j] / (prev_d - dsignalarray[j])) * dt);
    }
    prev_d = dsignalarray[j];
  }
  return 0;
}