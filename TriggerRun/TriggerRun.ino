//shows raw data with trigger and sg filter

#define High_threshold1 800
#define Low_threshold1 800
#define pin_input1 A2
#define arraysize 2000
#define dpin_in 3
#define dpin_out 6

int signalarray[arraysize];

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT);
  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
}

void loop() {
  if (digitalRead(dpin_in) == HIGH) {
    unsigned long start_time = micros();
    digitalWrite(dpin_out, HIGH);
    int value1 = analogRead(pin_input1);
    if (value1 > High_threshold1) {
      unsigned long time_peak = micros();
      int count = 0;
      do {
        if (count < arraysize) signalarray[count] = value1;
        value1 = analogRead(pin_input1);
        count++;
      } while (value1 > Low_threshold1);
      unsigned long end_time = micros();
      if (count > arraysize) count = arraysize;
      unsigned long peak_offset = peakfinder(count, end_time - time_peak);
      Serial.println((double)(time_peak - start_time + peak_offset) / 1000000.0, 6);
    }
    digitalWrite(dpin_out, LOW); //Change the trigger state back to LOW
  }
}

unsigned long peakfinder(int number, unsigned long duration) {
  unsigned long dt = duration / number;
  int prev_d = 0;
  for (int j = 6; j < (number - 7); j++) {
    int current_d = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]));
    if (current_d <= 0) {
      return (unsigned long)((j + (double)current_d / (prev_d - current_d)) * dt);
    }
    prev_d = current_d;
  }
  return 0;
}
