int random_variable;
int static_variable = 500;
#define REF_START_THRESHOLD 2900 // Reference peak start threshold
#define REF_END_THRESHOLD 2800 // Reference peak end threshold
#define SLAVE_START_THRESHOLD 1400 // Slave peak start threshold
#define SLAVE_END_THRESHOLD 1300 // Slave peak end threshold
#define pin_input1 A8 //single peak signal input
#define pin_input2 A8 //single peak signal input (same source for compatibility)
#define arraysize 2000 //size of the array for storing the data of the peaks
#define dpin_out 6//digital output for triggering function generator
#define dpin_in 3
#define alpha1_ref 0.6
#define alpha2_ref 0.50
#define pin_output1 DAC0 //feedback signal to 935nm ECDL piezo
#define pin_output1 DAC1 //feedback signal to 795nm DBR current

unsigned long t01, t02, t1, t2, start_time, end_time, tstartsweep, time_peak, timenow, period =50;
int signalarray[arraysize], dsignalarray[arraysize] = {};
int i, len, counter, Range, j;
double value1 , value2;
float alpha1, alpha2, error1, error2, totalT, control_output1, control_output2;
bool trigger, flag = HIGH;//Flag is for detecting the position of the peak.
bool indicator2 = LOW; //indicator is for indicating the presence of the 795 peak
bool lock;

//---------------------------------------------R-
//Define Servo Loop Variables for 795nm feedback
int sign2 = -1; //define the sign of PID parameters. 1 is positive and -1 is negative.
volatile float laser2_K_i = 1;
volatile float laser2_K_p = 0.7;
float laser2_error_signal_current;
float laser2_error_signal_prev;
float delta_laser2 = 0;
float laser2_control_signal = 0;
//----------------------------------------------

void setup() {
  Serial.begin(250000);
  //Serial.begin(9600);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  Range = (pow(2, 12) - 1) - 200;
}
void loop() {
  lock = digitalRead(dpin_in);
  if (digitalRead(dpin_in) == LOW) {
  start_time = micros();
  // triggerPin is read via dpin_out in TriggerCheck pattern
  trigger = HIGH;
  indicator2 = LOW;
  tstartsweep = millis();
  counter = 0;
  do {
    //Serial.println(counter);
    int sample = analogRead(pin_input1);
    timenow = millis();
    if (timenow - tstartsweep > period) { //Set the limit of time to sweep PZT
      trigger = LOW;
    }

    if (counter == 0 && sample > REF_START_THRESHOLD) { // first reference peak
      time_peak = micros();
      //Serial.println(time_peak-start_time);
      i = 0;
      do {
        signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > REF_END_THRESHOLD && i < arraysize);
      end_time = micros();
      len = i;
      t01 = time_peak-start_time + peakfinder(len, end_time - time_peak);
      counter = 1;
      continue;
    }

    if (counter == 1 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) { // first slave peak
      time_peak = micros();
      i = 0;
      do {
        signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > SLAVE_END_THRESHOLD && i < arraysize);
      end_time = micros();
      len = i;
      t2 = time_peak-start_time + peakfinder(len, end_time - time_peak);
      counter = 2;
      continue;
    }

    if (counter == 2 && sample > SLAVE_START_THRESHOLD && sample < REF_START_THRESHOLD) { // ignore second slave peak
      do {
        sample = analogRead(pin_input1);
      } while (sample > SLAVE_END_THRESHOLD && sample > 0);
      continue;
    }

    if (counter == 2 && sample > REF_START_THRESHOLD) { // second reference peak
      time_peak = micros();
      i = 0;
      do {
        signalarray[i] = sample;
        sample = analogRead(pin_input1);
        i++;
      } while (sample > REF_END_THRESHOLD && i < arraysize);
      end_time = micros();
      len = i;
      t02 = time_peak-start_time + peakfinder(len, end_time - time_peak);
      counter = 3;
      trigger = LOW;
      continue;
    }
                  
      
      

//
  } while (trigger == HIGH);
  
//Serial.println("delt1");
//error1 = (double)t1 - t01;
error2 = (double)t2 - t01;
totalT = (double)t02 - t01;
//--------------------------------------------------feedback for 795nm DBR
if (totalT > 0 && totalT < 160000 && error2 < totalT) {
    alpha2 = error2 / totalT; //scaled error signal.
    if (indicator2 == LOW || counter < 2) { //if there is no third peak, do not apply lock.
      laser2_control_signal = 0;
      control_output2 = (double)650 + Range / 2.0 * laser2_control_signal;
      analogWrite(DAC0, control_output2);
      Serial.println("No third peak");
    }
    else { //if there is the third peak, apply lock.
      else { // If the third peak is detected, apply lock.
      // 1. Calculate the current error (Difference between Target and Measured position)
      laser2_error_signal_current = alpha2_ref - (double)error2 / totalT;
      Serial.println(laser2_error_signal_current);

      // 2. Calculate the change (Delta) using PI logic
      // Proportional term reacts to the change in error
      // Integral term adds a correction based on the absolute error
      delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + 
                     sign2 * (laser2_K_i * laser2_error_signal_current);
      
      // 3. Update the cumulative control signal
      laser2_control_signal += delta_laser2;
      
      // 4. Convert math signal to DAC voltage (Centered at 2072.5)
      control_output2 = (double)2072.5 + Range / 2.0 * laser2_control_signal;

      // 5. SAFETY CLAMPING: Prevent the signal from going out of 12-bit bounds (0-4095)
      // This protects the DBR from current spikes if the peak is briefly lost
      if (control_output2 > 4095) {
          control_output2 = 4095;
          // Optional: laser2_control_signal -= delta_laser2; // Anti-windup adjustment
      } 
      if (control_output2 < 0) {
          control_output2 = 0;
      }

      // 6. Write the physical adjustment to the DBR current driver
      analogWrite(DAC1, control_output2);

      // Store the current error to use as 'previous' in the next iteration
      laser2_error_signal_prev = laser2_error_signal_current;
    }
    }
    //Serial.println(alpha2 * 10, 4);
  }

// In TriggerCheck pattern the trigger state is read from dpin_out; do not drive dpin_out here
  }
  else {
    Serial.println("Lock disengaged");
    control_output2 = (double)2072.5;
    analogWrite(DAC1, control_output2);
  }
}
//Serial.println("delt");
//Serial.println(t2-t01);



unsigned long peakfinder(int number, unsigned long duration) {//subfunction for finding peaks
  //Using SG filter to determine the time of the peak.
  unsigned long dt, peaktime;
  flag = HIGH;
  dt = duration / number;
  //Serial.println("time");
  //Serial.println(duration);
  //Serial.println("number");
  //Serial.println(number);
  for (int j = 6; j < (number - 7); j++) {
    //dsignalarray[j] = int(( 5 * signalarray[j + 5] + 4 * signalarray[j + 4] +3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3]- 4 * signalarray[j - 4] - 5 * signalarray[j - 5]));
    dsignalarray[j] = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]));
    
    //Serial.println((dsignalarray[j]));
    //analogWrite(DAC0, (dsignalarray[j] + 20000) * 0.08);
    //analogWrite(DAC0, (dsignalarray[j] + 1000) * 1.8);
    if (dsignalarray[j] <= 0 && flag) {
      //Serial.println("peaktime");
      
      flag = LOW;
      //peaktime = (j + dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;//uses the method of linear interpolation near the zero crossing to calculate the peaktime
      peaktime = (j+ dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;//uses the method of linear interpolation near the zero crossing to calculate the peaktime
      return peaktime;
      //Serial.println(j);
    }
  }
  //Serial.println(peaktime);
  //Serial.println(dsignalarray[j]);
  //Serial.println(dsignalarray[j-1]);
  //return peaktime;
}
