// Allan Variance Analysis for STCL Laser Locking System
// Measures frequency stability and linewidth error over time

#define pin_input1 A2
#define pin_input2 A4 
#define dpin_in 3
#define dpin_out 5
#define pin_output2 DAC1
#define arraysize 1000

// Thresholds and References
#define High_threshold1 500
#define Low_threshold1 400
#define High_threshold2 1000
#define Low_threshold2 700
#define alpha1_ref 0.6
#define alpha2_ref 0.50

// Allan Variance Parameters
#define MAX_MEASUREMENTS 5000     // Maximum number of frequency measurements to collect
#define NUM_TAU_VALUES 10          // Number of different tau values to analyze
#define MEASUREMENT_DURATION 14400000 // Duration to collect measurements (ms) - 4 hours

// Global Variables - Peak Detection
int signalarray[arraysize], dsignalarray[arraysize];
unsigned long t01, t02, t2, start_time, end_time, time_peak, tstartsweep, timenow;
unsigned long period = 50; // timeout period (ms), matched to UneditedCode
int counter, len, Range;
double error2, totalT;
bool flag;

// Global Variables - PID Control
int sign2 = -1;
volatile float laser2_K_i = 1;
volatile float laser2_K_p = 0.7;
float laser2_error_signal_current, laser2_error_signal_prev, laser2_control_signal;

// Global Variables - Allan Variance Analysis
unsigned long frequency_measurements[MAX_MEASUREMENTS];  // Measured frequencies (Hz scaled by 1000)
unsigned long time_intervals[MAX_MEASUREMENTS];          // Time intervals between peaks (microseconds)
int measurement_count = 0;
unsigned long analysis_start_time = 0;
bool is_analyzing = false;

// Allan Variance storage
double allan_variance_results[NUM_TAU_VALUES];
unsigned long tau_values[NUM_TAU_VALUES];

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  
  Range = (pow(2, 12) - 1) - 200;
  
  // Calculate tau values (in microseconds) for Allan variance analysis
  calculateTauValues();
  
  Serial.println("=== STCL Laser Allan Variance Analyzer ===");
  Serial.println("Starting peak detection and frequency measurement...");
  Serial.print("Collecting ");
  Serial.print(MEASUREMENT_DURATION / 1000);
  Serial.println(" seconds of data.");
}

unsigned long peakfinder(int number, unsigned long duration) {
  if (number < 13) return 0;
  unsigned long dt = duration / number;
  flag = HIGH;
  for (int j = 6; j < (number - 7); j++) {
    dsignalarray[j] = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 
                       3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - 
                       signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 
                       4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]));
    if (dsignalarray[j] <= 0 && flag) {
      flag = LOW;
      return (j + (float)dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;
    }
  }
  return 0;
}

void loop() {
  Serial.println("Loop running");
  // Start analysis window
  if (!is_analyzing) {
    is_analyzing = true;
    analysis_start_time = millis();
    measurement_count = 0;
    Serial.println("Analysis started.");
  }
  
  // Check if analysis window is complete
  if (millis() - analysis_start_time > MEASUREMENT_DURATION) {
    // Calculate and display Allan variance
    if (measurement_count >= 2) {
      calculateAllanVariance();
      printResults();
    }
    is_analyzing = false;
    delay(5000); // Wait before starting next analysis
    return;
  }
  
  // Normal peak detection and frequency measurement
  if (digitalRead(dpin_in) == LOW) {
    if (digitalRead(dpin_out) == HIGH) {
      start_time = micros();
      bool indicator2 = LOW;
      tstartsweep = millis();
      counter = 0;
      bool sweep_active = true;
      bool timed_out = false;

      do {
        timenow = millis();
        if (timenow - tstartsweep > period) { timed_out = true; sweep_active = false; }
        int value1 = analogRead(pin_input1);
        int value2 = analogRead(pin_input2);
        
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
          if (counter >= 3) break;
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
          t2 = (time_peak - start_time) + peakfinder(i, end_time - time_peak);
          counter++;
          if (counter >= 3) break;
        }
      } while (sweep_active);

      if (counter < 3 || !indicator2) {
        Serial.println("Not all three peaks found");
      }

      if (timed_out) {
        Serial.println("Timeout - resetting after sweep");
        // Reset PID output to safe default after timeout
        laser2_control_signal = 0;
        analogWrite(pin_output2, 2072);
      }

      error2 = (double)t2 - t01;
      totalT = (double)t02 - t01;

      // Store measurement if valid
      if (totalT > 0 && totalT < 160000 && counter >= 2 && indicator2 && measurement_count < MAX_MEASUREMENTS) {
        // Calculate frequency from period
        unsigned long period_us = (unsigned long)totalT;
        time_intervals[measurement_count] = period_us;
        frequency_measurements[measurement_count] = 1000000000 / period_us; // Frequency in Hz (scaled by 1000 for precision)
        measurement_count++;
        
        // Apply PID control for locking
        laser2_error_signal_current = alpha2_ref - (error2 / totalT);
        float delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + 
                             sign2 * (laser2_K_i * laser2_error_signal_current);
        laser2_control_signal += delta_laser2;
        laser2_error_signal_prev = laser2_error_signal_current;

        float control_output2 = 2072.5 + (Range / 2.0) * laser2_control_signal;
        analogWrite(pin_output2, (int)control_output2);
      }
    }
  } else {
    Serial.println("Lock disengaged");
    analogWrite(pin_output2, 2072);
  }
}

void calculateTauValues() {
  // Generate logarithmically spaced tau values (in microseconds)
  // From 1 microsecond to ~1 second
  for (int i = 0; i < NUM_TAU_VALUES; i++) {
    tau_values[i] = (unsigned long)(1000 * pow(10, (float)i * 3.0 / NUM_TAU_VALUES)); // 1us to 1000000us
  }
}

void calculateAllanVariance() {
  for (int tau_idx = 0; tau_idx < NUM_TAU_VALUES; tau_idx++) {
    unsigned long tau = tau_values[tau_idx];
    int num_samples_in_tau = tau / (time_intervals[0]); // Approximate number of periods in tau
    
    if (num_samples_in_tau < 1) num_samples_in_tau = 1;
    
    double sum_squared_diff = 0;
    int valid_pairs = 0;
    
    // Calculate Allan variance: σ²ₐ(τ) = (1/2) * <(ỹ_{n+1} - ỹₙ)²>
    // Where ỹₙ is frequency averaged over tau
    for (int n = 0; n < (measurement_count - num_samples_in_tau); n++) {
      unsigned long avg_freq_1 = 0;
      unsigned long avg_freq_2 = 0;
      
      // Average frequencies over tau for first group
      for (int i = 0; i < num_samples_in_tau && (n + i) < measurement_count; i++) {
        avg_freq_1 += frequency_measurements[n + i];
      }
      avg_freq_1 /= num_samples_in_tau;
      
      // Average frequencies over tau for second group
      for (int i = 0; i < num_samples_in_tau && (n + num_samples_in_tau + i) < measurement_count; i++) {
        avg_freq_2 += frequency_measurements[n + num_samples_in_tau + i];
      }
      avg_freq_2 /= num_samples_in_tau;
      
      long freq_diff = (long)avg_freq_2 - (long)avg_freq_1;
      sum_squared_diff += (double)freq_diff * freq_diff;
      valid_pairs++;
    }
    
    if (valid_pairs > 0) {
      // Normalize frequency differences (convert from scaled units back to Hz)
      double normalized_sum = sum_squared_diff / (1e12); // Account for Hz scaling
      allan_variance_results[tau_idx] = normalized_sum / (2.0 * valid_pairs);
    } else {
      allan_variance_results[tau_idx] = 0;
    }
  }
}

void printResults() {
  Serial.println("\n=== ALLAN VARIANCE RESULTS ===");
  Serial.println("Tau (us) | Allan Variance | Allan Dev");
  Serial.println("---------|----------------|----------");
  
  for (int i = 0; i < NUM_TAU_VALUES; i++) {
    double allan_dev = sqrt(allan_variance_results[i]);
    
    Serial.print(tau_values[i]);
    Serial.print(" | ");
    Serial.print(allan_variance_results[i], 6);
    Serial.print(" | ");
    Serial.println(allan_dev, 6);
  }
  
  Serial.print("\nTotal measurements collected: ");
  Serial.println(measurement_count);
  
  // Find minimum Allan deviation (best stability point)
  int min_idx = 0;
  double min_allan_dev = sqrt(allan_variance_results[0]);
  for (int i = 1; i < NUM_TAU_VALUES; i++) {
    double allan_dev = sqrt(allan_variance_results[i]);
    if (allan_dev < min_allan_dev) {
      min_allan_dev = allan_dev;
      min_idx = i;
    }
  }
  
  Serial.println("\n=== STABILITY SUMMARY ===");
  Serial.print("Best stability at tau = ");
  Serial.print(tau_values[min_idx]);
  Serial.print(" us with Allan Dev = ");
  Serial.println(min_allan_dev, 6);
  
  Serial.println("\nRestarting analysis in 5 seconds...\n");
}
