// Pins and Constants
#define pin_input1 A2
#define pin_input2 A4 
#define dpin_in 2       // External trigger pin (Ensure this supports interrupts)
#define dpin_out 6      // Digital output for triggering function generator 
#define pin_output2 DAC1 // Feedback signal to 795nm DBR current
#define arraysize 2000

// Thresholds and References
#define High_threshold1 500
#define Low_threshold1 400
#define High_threshold2 1000
#define Low_threshold2 700
#define alpha1_ref 0.6  // 
#define alpha2_ref 0.50

// Global Variables
// Trigger state will be read via polling from `dpin_in` in the main loop
int signalarray[arraysize], dsignalarray[arraysize];
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep, timenow; // [cite: 46]
unsigned long period = 50; // Sweep period limit in ms
int counter, len, Range;
double error2, totalT;
bool trigger, flag; // [cite: 48]

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
  pinMode(dpin_in, INPUT);
  
  pinMode(dpin_out, OUTPUT); // [cite: 51]
  digitalWrite(dpin_out, LOW); // [cite: 51]
  
  Range = (pow(2, 12) - 1) - 200;
}

void loop() {
  // Read the trigger pin via polling (two-pin system: dpin_in + dpin_out)
  bool lock = digitalRead(dpin_in);

  if (lock) {
    start_time = micros();
    digitalWrite(dpin_out, HIGH);
    trigger = HIGH; 
    bool indicator2 = LOW;
    tstartsweep = millis();
    counter = 0;

    do {
      int value1 = analogRead(pin_input1);
      int value2 = analogRead(pin_input2);
      timenow = millis(); // [cite: 54]

      // Trigger timeout logic [cite: 55]
      if (timenow - tstartsweep > period) { 
        trigger = LOW;
      }
      
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
          t01 = (time_peak - start_time) + peakfinder(len, end_time - time_peak); // [cite: 59]
        } else if (counter == 2) {
          t02 = (time_peak - start_time) + peakfinder(len, end_time - time_peak); // [cite: 60]
        }
        counter++;
      }

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
        t2 = (time_peak - start_time) + peakfinder(i, end_time - time_peak); // [cite: 64]
        counter++;
      }
    } while (trigger == HIGH); // Loop based on trigger status [cite: 65]

    error2 = (double)t2 - t01;
    totalT = (double)t02 - t01;

    if (totalT > 0 && totalT < 160000 && counter >= 2 && indicator2) {
      laser2_error_signal_current = alpha2_ref - (error2 / totalT);
      float delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + 
                           sign2 * (laser2_K_i * laser2_error_signal_current);
      laser2_control_signal += delta_laser2;
      laser2_error_signal_prev = laser2_error_signal_current;

      float control_output2 = 2072.5 + (Range / 2.0) * laser2_control_signal;
      analogWrite(pin_output2, (int)control_output2);
      
      Serial.print("Error:"); Serial.println(laser2_error_signal_current);
    }
    digitalWrite(dpin_out, LOW); // Manual trigger end [cite: 72]
  } else {
    Serial.println("Lock disengaged"); // [cite: 73]
    analogWrite(pin_output2, 2072); 
  }
}

// ISR removed: using polling on `dpin_in` for trigger state

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
