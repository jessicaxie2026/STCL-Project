// Pins and Constants
#define pin_input1 A8
#define pin_input2 A8 
#define dpin_in 8        // Manual lock switch
#define dpin_out 12      // Trigger input from function generator
#define pin_output2 DAC1 // Feedback signal to 795nm DBR current
#define arraysize 2000

// Thresholds and References
// Reference peaks are larger and use the higher threshold values.
#define REF_START_THRESHOLD 1250
#define REF_END_THRESHOLD 1200

// Slave peaks are smaller and use the lower threshold values.
#define SLAVE_START_THRESHOLD 900
#define SLAVE_END_THRESHOLD 885

#define alpha1_ref 0.6  
#define alpha2_ref 0.50

// Global Variables
// Trigger state will be read via polling from `dpin_in` in the main loop
int signalarray[arraysize], dsignalarray[arraysize];
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep, timenow; // [cite: 46]
unsigned long period = 55; // timeout period (ms), matched to TriggerErrorTest
int counter, len, Range;
double error2, totalT;
float offset = 3733.0;
float error = 0.0;
const float DAC_MIN_COUNTS = 3.1f / 3.3f * 4095.0f;
const float DAC_MAX_COUNTS = 3.3f / 3.3f * 4095.0f;
// Using TriggerCheck-style polling: dpin_in acts as lock (INPUT_PULLUP)
bool flag; 
bool sweep_active = false;
bool prev_trigger_state = false;
bool indicator2 = false;

// PID Variables 
int sign2 = -1;
volatile float laser2_K_i = 1;
volatile float laser2_K_p = 0.7;
float laser2_error_signal_current, laser2_error_signal_prev, laser2_control_signal;

void setup() {
  Serial.begin(115200); // [cite: 50]
  analogReadResolution(12);
  analogWriteResolution(12);
  
  // --- EXTERNAL TRIGGER SETUP ---
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  // trigger read via dpin_out per TriggerCheck mapping
  
  Range = (pow(2, 12) - 1) - 200;
  Serial.println("Setup complete");
}

unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  flag = HIGH;
  for (int j = 6; j < (number - 7); j++) {
    dsignalarray[j] = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - 
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6])); // [cite: 76]
    if (dsignalarray[j] <= 0 && flag) {
      flag = LOW;
      return (j + (float)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt; // [cite: 77]
    }
  }
  return 0;
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
      indicator2 = false;
      sweep_active = true;
    }
    prev_trigger_state = trigger_now;
    return;
  }

  if (millis() - tstartsweep > period) {
    sweep_active = false;
    prev_trigger_state = trigger_now;
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

    end_time = micros();
    len = i;
    t01 = time_peak - start_time + peakfinder(len, end_time - time_peak);
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

      end_time = micros();
      t2 = current_offset + peakfinder(i, end_time - time_peak);
      counter++;
    }
  }

  else if (counter == 2 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
    int i = 0;
    do {
      sample = analogRead(pin_input1);
      i++;
    } while (sample > SLAVE_END_THRESHOLD && i < arraysize);
    counter++;
  }

  else if (counter == 3 && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      if (i < arraysize) signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    end_time = micros();
    len = i;
    t02 = time_peak - start_time + peakfinder(len, end_time - time_peak);
    counter++;
  }

  if (counter >= 4) {
    sweep_active = false;

    if (t2 > t01 && t2 < t02) {
      error2 = (double)t2 - t01;
      totalT = (double)t02 - t01;

      if (totalT > 0 && totalT < 160000) {
        laser2_error_signal_current = alpha2_ref - (error2 / totalT);
        float delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + 
                             sign2 * (laser2_K_i * laser2_error_signal_current);
        laser2_control_signal += delta_laser2;
        laser2_error_signal_prev = laser2_error_signal_current;

        error = offset + (Range / 2.0) * laser2_control_signal;
        if (error > DAC_MAX_COUNTS) error = DAC_MAX_COUNTS;
        if (error < DAC_MIN_COUNTS) error = DAC_MIN_COUNTS;
        analogWrite(pin_output2, (int)error);

        Serial.print(t01); Serial.print(", ");
        Serial.print(t2); Serial.print(", ");
        Serial.print(t02); Serial.print(", Lock Error: ");
        Serial.println(laser2_error_signal_current, 6);
      }
    } else {
      counter = 0;
      Serial.println("Glitchy sequence");
    }
  }

  prev_trigger_state = trigger_now;
}
