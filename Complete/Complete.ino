int random_variable;
int static_variable = 500;
#define High_threshold1 500 //High threshold for starting reading signal from peaks
#define Low_threshold1 400 //Low threshold for stopping reading signal from peaks
#define High_threshold2 1000 //High threshold for starting reading signal from peaks
#define Low_threshold2 700 //Low threshold for stopping reading signal from peaks
#define pin_input1 A2 //input signal from PD1 (D2 and 935nm)
#define pin_input2 A4 //input signal from PD2 (795nm)
#define arraysize 2000 //size of the array for storing the data of the peaks
#define dpin_out 5//digital output for triggering function generator
#define dpin_in 3
#define alpha1_ref 0.6
#define alpha2_ref 0.50
#define pin_output1 DAC0 //feedback signal to 935nm ECDL piezo
#define pin_output2 DAC1 //feedback signal to 795nm DBR current

unsigned long t01, t02, t1, t2, start_time, end_time, tstartsweep, time_peak, timenow, period =50;
int signalarray[arraysize], dsignalarray[arraysize] = {};
int i, len, counter, Range, j;
double value1 , value2;
float alpha1, alpha2, error1, error2, totalT, control_output1, control_output2;
// Using TriggerCheck-style polling: dpin_in acts as lock (INPUT_PULLUP)
bool flag = HIGH;//Flag is for detecting the position of the peak.
bool indicator2 = LOW; //indicator is for indicating the presence of the 795 peak
bool lock;

//----------------------------------------------
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
  Serial.begin(115200);
  //Serial.begin(9600);
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(dpin_in, INPUT_PULLUP);
  pinMode(dpin_out, INPUT);
  // read triggerPin via dpin_out in loop
  Range = (pow(2, 12) - 1) - 200;
  Serial.println("Setup complete");
}

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
  return 0;
}

void loop() {
  Serial.println("Loop running");
  if (digitalRead(dpin_in) == LOW) {
    if (digitalRead(dpin_out) == HIGH) {
      start_time = micros();
      indicator2 = LOW;
      tstartsweep = millis();
      counter = 0;
      bool sweep_active = true;
      bool timed_out = false;
  do {
    timenow = millis();
    if (timenow - tstartsweep > period) { timed_out = true; sweep_active = false; }
    //Serial.println(counter);
    value1 = analogRead(pin_input1);
    value2 = analogRead(pin_input2);
    //Serial.println(value1);
    if (value1 > High_threshold1 ) {//starting point of the peaks from PD1
      
      
      time_peak = micros();
      //Serial.println(time_peak-start_time);
      i = 0;
      do {
        //Serial.println(value1);
        signalarray[i] = value1;
        //analogWrite(DAC1, value1);
        value1 = analogRead(pin_input1);
        i++;
        //Serial.println(value1);
      } while(value1 > Low_threshold1); 
      end_time = micros();
      len = i;
      if (counter == 0) {
        //t01 = start_time + peakfinder(len, end_time - start_time);
        t01 = time_peak-start_time + peakfinder(len, end_time - time_peak);
        //Serial.println("del");
        //Serial.println(t01);
        //trigger = LOW;
      }
      
      
      if (counter == 2) {
        //t02 = start_time + peakfinder(len, end_time - start_time);
        t02 = time_peak-start_time+ peakfinder(len, end_time - time_peak);
        //Serial.println(t02);
      }
      counter++;  
      if (counter >= 3) break;
      }
      if (counter == 1) {
        if (value2 > High_threshold2 && indicator2 == LOW) { //starting point of the 795nm peak, counter>0 requires that the peak between two reference peaks.
        indicator2 = HIGH;
        time_peak = micros();
        i = 0;
        do {
          signalarray[i] = value2;
          //analogWrite(DAC1, value2);
          //Serial.println(value2);
          value2 = analogRead(pin_input2);
          i++;
        } while (value2 > Low_threshold2);
        end_time = micros();
        len = i;
        t2 =time_peak-start_time+ peakfinder(len, end_time - time_peak);
        //Serial.println(peakfinder(len, end_time - time_peak));
        //Serial.println(t2);
        //peakfinder(len, end_time - time_peak);
        counter++;
    }
      }
                  
      
      

//
  } while (sweep_active);

  if (timed_out) {
    Serial.println("Timeout - resetting after sweep");
    // Reset control outputs to safe defaults after timeout
    laser2_control_signal = 0;
    control_output2 = (double)2072.5;
    analogWrite(DAC1, control_output2);
  }
  
//Serial.println("delt1");
//error1 = (double)t1 - t01;
error2 = (double)t2 - t01;
totalT = (double)t02 - t01;
//--------------------------------------------------feedback for 795nm DBR
if (totalT > 0 && totalT < 160000 && error2 < totalT) {
    alpha2 = error2 / totalT; //scaled error signal.
    if (!indicator2 || counter < 3) { //if not all three peaks found, do not apply lock.
      laser2_control_signal = 0;
      control_output2 = (double)2072.5 + Range / 2.0 * laser2_control_signal;
      analogWrite(DAC1, control_output2);
    }
    else { // if there is the third peak, apply lock.
      laser2_error_signal_current = alpha2_ref - (double)error2 / totalT;
    

      // --- 2. PID CALCULATION ---
      delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + sign2 * (laser2_K_i * laser2_error_signal_current);
      
      laser2_control_signal += delta_laser2;

      // --- 3. ANTI-WINDUP CAP (THE "CAP") ---
      // This prevents the integrator from "running away" if the laser loses lock.
      // 0.5 is a safe limit for your signal range, adjust if needed.
      if (laser2_control_signal > 0.5) laser2_control_signal = 0.5;
      if (laser2_control_signal < -0.5) laser2_control_signal = -0.5;

      laser2_error_signal_prev = laser2_error_signal_current;
      
      // --- 4. OUTPUT TO DAC ---
      control_output2 = (double)2072.5 + Range / 2.0 * laser2_control_signal;
      analogWrite(DAC1, control_output2);

      // --- 1. CSV DATA LOGGING ---
      // This prints: Time, Alpha_Ref, Measured_Alpha, Error, DAC_Value
      Serial.print(millis());
      Serial.print(",");
      Serial.print(alpha2_ref, 4);
      Serial.print(",");
      Serial.print((double)error2 / totalT, 4);
      Serial.print(",");
      Serial.print(laser2_error_signal_current, 6);
      Serial.print(",");
      Serial.println(control_output2); 
    }
    //Serial.println(alpha2 * 10, 4);
  }

  if (counter < 3 || !indicator2) {
    Serial.println("Not all three peaks found");
  }

    }
  } else {
    Serial.println("Lock disengaged");
    control_output2 = (double)2072.5;
    analogWrite(DAC1, control_output2);
  }
}
//Serial.println("delt");
//Serial.println(t2-t01);



void triggerISR() {
  // Removed: trigger read is handled by polling dpin_out
}
