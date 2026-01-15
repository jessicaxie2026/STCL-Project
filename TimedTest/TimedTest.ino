// --- Configuration and Thresholds ---
#define High_threshold1 850
#define Low_threshold1 850

#define pin_input1 A2
#define dpin_in 3       // Input pin for external trigger/lock
#define dpin_out 6      // Digital output for triggering function generator
#define arraysize 2000  // Size of the array for storing peak data

// --- Global Variables ---
unsigned long t01, start_time, end_time, time_peak, tstartsweep;
unsigned long period = 50; // Sweep period limit in ms
int signalarray[arraysize], dsignalarray[arraysize];
bool flag = HIGH; // Flag for detecting peak position
bool trigger;     // Sweep control flag

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(dpin_in, INPUT);
  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
}

void loop() {
  // Check if the lock/trigger pin is HIGH to start the cycle
  bool lock = digitalRead(dpin_in);

  if (lock) {
    start_time = micros();
    digitalWrite(dpin_out, HIGH);
    trigger = HIGH;
    tstartsweep = millis();
    bool peak_captured = false;

    // Loop until sweep period expires
    do {
      int value1 = analogRead(pin_input1);
      unsigned long timenow = millis();

      // Timeout safety
      if (timenow - tstartsweep > period) {
        trigger = LOW;
      }

      // Detect peak on Channel 1
      if (value1 > High_threshold1 && !peak_captured) {
        time_peak = micros();
        int i = 0;
        do {
          if (i < arraysize) signalarray[i] = value1;
          value1 = analogRead(pin_input1);
          i++;
        } while (value1 > Low_threshold1);

        end_time = micros();

        // Calculate exact peak time using SG filter
        t01 = (time_peak - start_time) + peakfinder(i, end_time - time_peak);
        peak_captured = true;
      }
    } while (trigger == HIGH);

    // --- Output Results ---
    if (peak_captured) {
      Serial.print("Peak Time (us): ");
      Serial.println(t01);
    } else {
      Serial.println("Error: No peak detected within timeout.");
    }

    digitalWrite(dpin_out, LOW); //Change the trigger state back to LOW
  } else {
    Serial.println("Lock disengaged");
  }
}

// Savitzky-Golay Filter for peak center detection
unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 14) return 0;

  unsigned long dt = duration / number;
  flag = HIGH;

  for (int j = 6; j < (number - 7); j++) {
    // 13-point Savitzky-Golay derivative filter
    dsignalarray[j] = (6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] +
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] -
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] -
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]);

    // Zero-crossing detection for the derivative (peak center)
    if (dsignalarray[j] <= 0 && flag) {
      flag = LOW;
      // Linear interpolation for sub-sample precision
      return (unsigned long)((j + (double)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt);
    }
  }
  return 0;
}
