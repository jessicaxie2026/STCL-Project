//shows raw data with trigger and sg filter

#define REF_START_THRESHOLD 2900
#define REF_END_THRESHOLD 2800
#define pin_input1 A8
#define arraysize 2000
#define dpin_in 3
#define dpin_out 6

int signalarray[arraysize];
unsigned long start_time = 0;
unsigned long tstartsweep = 0;
bool sweep_active = false;
bool prev_trigger_state = false;

// Using TriggerCheck-style polling: dpin_in acts as lock (INPUT_PULLUP)

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
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
      tstartsweep = millis();
      sweep_active = true;
      Serial.println("Sweep start: trigger fell");
    }
    prev_trigger_state = trigger_now;
    return;
  }

  int sample = analogRead(pin_input1);
  if (millis() - tstartsweep > 50) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  if (sample > REF_START_THRESHOLD) {
    unsigned long time_peak = micros();
    int count = 0;
    do {
      if (count < arraysize) signalarray[count] = sample;
      sample = analogRead(pin_input1);
      count++;
    } while (sample > REF_END_THRESHOLD && (millis() - tstartsweep) <= 50);
    unsigned long end_time = micros();
    if (count > arraysize) count = arraysize;
    unsigned long peak_offset = peakfinder(count, end_time - time_peak);
    Serial.println((double)(time_peak - start_time + peak_offset) / 1000000.0, 6);
    sweep_active = false;
  }

  prev_trigger_state = trigger_now;
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

 
