// Pins and Constants
#define pin_input1 A2
#define pin_input2 A4 
#define dpin_in 2       // External trigger pin (Ensure this supports interrupts)
#define pin_output2 DAC1 // Feedback signal to 795nm DBR current
#define arraysize 2000

// Thresholds and References
#define High_threshold1 500
#define Low_threshold1 400
#define High_threshold2 1000
#define Low_threshold2 700
#define alpha2_ref 0.50

// Global Variables
volatile bool trigger_active = false; // Shared between ISR and main loop [cite: 3]
int signalarray[arraysize], dsignalarray[arraysize];
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep;
int counter, len, Range;
double error2, totalT;
bool flag;

// PID Variables [cite: 15, 16, 17]
int sign2 = -1;
volatile float laser2_K_i = 1;
volatile float laser2_K_p = 0.7;
float laser2_error_signal_current, laser2_error_signal_prev, laser2_control_signal;

void setup() {
  Serial.begin(115200); // Higher baud rate for timing accuracy [cite: 4]
  analogReadResolution(12);
  analogWriteResolution(12);
  
  pinMode(dpin_in, INPUT);
  // Attach interrupt for external trigger [cite: 5]
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  
  trigger_active = digitalRead(dpin_in); // Initial state [cite: 6]
  Range = (pow(2, 12) - 1) - 200;
}

void loop() {
  // Use the volatile bool from the ISR instead of digitalRead [cite: 3, 19]
  if (trigger_active) {
    start_time = micros();
    tstartsweep = millis();
    counter = 0;
    bool indicator2 = LOW;

    // Process peaks while trigger is active (or until timeout)
    do {
      int value1 = analogRead(pin_input1);
      int value2 = analogRead(pin_input2);

      // Timeout safety (50ms period) [cite: 11, 21]
      if (millis() - tstartsweep > 50) break;

      // Peak Detection Logic [cite: 22, 23]
      if (value1 > High_threshold1) {
        time_peak = micros();
        int i = 0;
        do {
          signalarray[i] = value1;
          value1 = analogRead(pin_input1);
          i++;
        } while (value1 > Low_threshold1 && i < arraysize);
        
        end_time = micros();
        len = i;

        if (counter == 0) {
          t01 = (time_peak - start_time) + peakfinder(len, end_time - time_peak);
        } else if (counter == 2) {
          t02 = (time_peak - start_time) + peakfinder(len, end_time - time_peak);
        }
        counter++;
      }

      // Second Laser Peak Detection [cite: 29, 30]
      if (counter == 1 && value2 > High_threshold2 && !indicator2) {
        indicator2 = HIGH;
        time_peak = micros();
        int i = 0;
        do {
          signalarray[i] = value2;
          value2 = analogRead(pin_input2);
          i++;
        } while (value2 > Low_threshold2 && i < arraysize);
        
        end_time = micros();
        t2 = (time_peak - start_time) + peakfinder(i, end_time - time_peak);
        counter++;
      }
    } while (trigger_active); // Controlled by external interrupt [cite: 7]

    // PID Calculations and Serial Output
    error2 = (double)t2 - t01;
    totalT = (double)t02 - t01;

    if (totalT > 0 && totalT < 160000 && counter >= 2 && indicator2) {
      // PID Loop [cite: 39, 40, 41]
      laser2_error_signal_current = alpha2_ref - (error2 / totalT);
      float delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + 
                           sign2 * (laser2_K_i * laser2_error_signal_current);
      laser2_control_signal += delta_laser2;
      laser2_error_signal_prev = laser2_error_signal_current;

      float control_output2 = 2072.5 + (Range / 2.0) * laser2_control_signal;
      analogWrite(pin_output2, (int)control_output2);

      // Requested Outputs: Times and Error [cite: 34, 35]
      Serial.print("t01:"); Serial.print(t01);
      Serial.print(" t02:"); Serial.print(t02);
      Serial.print(" t2:"); Serial.print(t2);
      Serial.print(" Error:"); Serial.println(laser2_error_signal_current);
    }
  } else {
    // Lock Disengaged [cite: 45]
    analogWrite(pin_output2, 2072); 
  }
}

// ISR for external trigger [cite: 5, 7]
void triggerISR() {
  trigger_active = digitalRead(dpin_in);
}

// Savitzky-Golay Filter for Peak Finding [cite: 46, 48, 51]
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  flag = HIGH;
  for (int j = 6; j < (number - 7); j++) {
    dsignalarray[j] = (6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - 
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]);
    
    if (dsignalarray[j] <= 0 && flag) {
      flag = LOW;
      return (j + (float)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;
    }
  }
  return 0;
}
