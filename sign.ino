#define pin_input1 A8
#define pin_output DAC0

#define REF_START_THRESHOLD 1250
#define REF_END_THRESHOLD 1200
#define SLAVE_START_THRESHOLD 900
#define SLAVE_END_THRESHOLD 885
#define alpha2_ref 0.50
#define arraysize 2000

int output = 2288;
int signalarray[arraysize];
bool running = false;

unsigned long t01 = 0;
unsigned long t2 = 0;
unsigned long t02 = 0;

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(pin_output, OUTPUT);

  output = constrain(output, 1150, 4095);
  analogWrite(pin_output, output);

  Serial.println("Sign test: write output, then print peak-system error.");
  Serial.println("Press 's' to start, 'q' to stop");
}

void loop() {
  static unsigned long lastStepMs = 0;
  static bool firstPass = true;

  // Check for serial commands
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
    delay(100);
    return;
  }

  if (firstPass) {
    firstPass = false;
    lastStepMs = millis();
  }

  if (millis() - lastStepMs >= 5000UL) {
    output += 13;
    if (output > 4095) {
      output = 4095;
    }
    lastStepMs = millis();
    Serial.print("New output = ");
    Serial.println(output);
  }

  int writeValue = constrain(output, 1150, 4095);
  analogWrite(pin_output, writeValue);

  double error = measurePeakError();

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

  delay(100);
}

double measurePeakError() {
  unsigned long sweepStart = micros();
  unsigned long sweepTimeout = millis() + 200;
  int counter = 0;
  int sample = 0;

  t01 = 0;
  t2 = 0;
  t02 = 0;

  while (millis() < sweepTimeout) {
    sample = analogRead(pin_input1);

    if (counter == 0 && sample > REF_START_THRESHOLD) {
      unsigned long time_peak = micros();
      int i = 0;
      do {
        signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > REF_END_THRESHOLD && i < arraysize);

      t01 = time_peak - sweepStart + peakfinder(i, micros() - time_peak);
      counter = 1;
      continue;
    }

    if (counter == 1 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
      unsigned long time_peak = micros();
      int i = 0;
      do {
        signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > SLAVE_END_THRESHOLD && i < arraysize);

      t2 = time_peak - sweepStart + peakfinder(i, micros() - time_peak);
      counter = 2;
      continue;
    }

    if (counter == 2 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
      do {
        sample = analogRead(pin_input1);
      } while (sample > SLAVE_END_THRESHOLD && sample > 0);
      continue;
    }

    if (counter == 2 && sample > REF_START_THRESHOLD) {
      unsigned long time_peak = micros();
      int i = 0;
      do {
        signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > REF_END_THRESHOLD && i < arraysize);

      t02 = time_peak - sweepStart + peakfinder(i, micros() - time_peak);
      break;
    }
  }

  if (t2 > t01 && t2 < t02 && t02 > t01) {
    double alpha2 = (double)(t2 - t01) / (double)(t02 - t01);
    return alpha2_ref - alpha2;
  }

  return 9999.0;
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

    if (dsignalarray[j] <= 0 && j > 6) {
      return (unsigned long)((j + (double)dsignalarray[j] / (prev_d - dsignalarray[j])) * dt);
    }
    prev_d = dsignalarray[j];
  }
  return 0;
}