// Pins and Constants
#define pin_input1 A8
#define pin_input2 A8 
#define dpin_in 8       // External trigger pin (Ensure this supports interrupts)
#define dpin_out 10      // Digital output for triggering function generator 
#define pin_output2 DAC1 // Feedback signal to 795nm DBR current
#define arraysize 2000

// Thresholds and References
// Reference peaks are larger and use the higher threshold values.
#define REF_START_THRESHOLD 2900
#define REF_END_THRESHOLD 2800

// Slave peaks are smaller and use the lower threshold values.
#define SLAVE_START_THRESHOLD 1400
#define SLAVE_END_THRESHOLD 1300

#define alpha1_ref 0.6  
#define alpha2_ref 0.50

// Global Variables
// Trigger state will be read via polling from `dpin_in` in the main loop
int signalarray[arraysize], dsignalarray[arraysize];
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep, timenow; // [cite: 46]
unsigned long period = 50; // timeout period (ms), matched to UneditedCode
int counter, len, Range;
double error2, totalT;
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
      Serial.println("Sweep start: trigger fell");
    }
    prev_trigger_state = trigger_now;
    return;
  }

  bool timed_out = false;
  timenow = millis();
  if (timenow - tstartsweep > period) {
    timed_out = true;
    sweep_active = false;
    prev_trigger_state = trigger_now;
    return;
  }

  int sample = analogRead(pin_input1);

  // Detect first reference peak
  if (counter == 0 && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    end_time = micros();
    len = i;
    t01 = (time_peak - start_time) + peakfinder(len, end_time - time_peak);
    counter = 1;
    return;
  }

  // Detect first slave peak only after the first reference peak
  if (counter == 1 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > SLAVE_END_THRESHOLD && i < arraysize);

    end_time = micros();
    t2 = (time_peak - start_time) + peakfinder(i, end_time - time_peak);
    counter = 2;
    return;
  }

  // Ignore the second slave peak completely, do not increment counter for it
  if (counter == 2 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) {
    // consume the second slave peak and continue waiting for the second reference peak
    int i = 0;
    do {
      sample = analogRead(pin_input1);
      i++;
    } while (sample > SLAVE_END_THRESHOLD && i < arraysize);
    return;
  }

  // Detect second reference peak only after first reference + first slave
  if (counter == 2)
  Serial.println("Loop running"); && sample > REF_START_THRESHOLD) {
    time_peak = micros();
    int i = 0;
    do {
      signalarray[i] = sample;
      sample = analogRead(pin_input1);
      i++;
    } while (sample > REF_END_THRESHOLD && i < arraysize);

    end_time = micros();
    len = i;
    t02 = (time_peak - start_time) + peakfinder(len, end_time - time_peak);
    counter = 3;
    sweep_active = false;
    return;
  }

  prev_trigger_state = trigger_now;

  if (!sweep_active) {
    if (counter < 3) {
      Serial.println("Not all three peaks found");
    }

    if (timed_out) {
      Serial.println("Timeout - resetting after sweep");
      laser2_control_signal = 0;
      analogWrite(pin_output2, 2072);
    }

    error2 = (double)t2 - t01;
    totalT = (double)t02 - t01;

    if (totalT > 0 && totalT < 160000 && counter >= 2) {
      laser2_error_signal_current = alpha2_ref - (error2 / totalT);
      float delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + 
                           sign2 * (laser2_K_i * laser2_error_signal_current);
      laser2_control_signal += delta_laser2;
      laser2_error_signal_prev = laser2_error_signal_current;

      float control_output2 = 2072.5 + (Range / 2.0) * laser2_control_signal;
      analogWrite(pin_output2, (int)control_output2);

      Serial.print("Error:"); Serial.println(laser2_error_signal_current);
      Serial.println("Control Signal:"); Serial.println(control_output2);
    }
  }
}
