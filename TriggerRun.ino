#define High_threshold1 800 //High threshold for starting reading signal from peaks
#define Low_threshold1 800 //Low threshold for stopping reading signal from peaks
#define pin_input1 A2 //input signal from PD1 (D2 and 935nm)
#define arraysize 2000 //size of the array for storing the data of the peaks
#define dpin_in 2

int signalarray[arraysize];
volatile bool trigger_active = false;

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT);
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  trigger_active = digitalRead(dpin_in);
}
void loop() {
  if (trigger_active) {
    unsigned long start_time = micros();
    while (trigger_active) {
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
        while (trigger_active) {}
      }
    }
  }
}



unsigned long peakfinder(int number, unsigned long duration) {//subfunction for finding peaks
  //Using SG filter to determine the time of the peak.
  unsigned long dt = duration / number;
  int prev_d = 0;
  for (int j = 6; j < (number - 7); j++) {
    int current_d = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]));
    
    if (current_d <= 0) {
      return (unsigned long)((j + (double)current_d / (prev_d - current_d)) * dt); //uses the method of linear interpolation near the zero crossing to calculate the peaktime
    }
    prev_d = current_d;
  }
  return 0;
}

void triggerISR() {
  trigger_active = digitalRead(dpin_in);
}
