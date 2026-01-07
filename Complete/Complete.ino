int random_variable;
int static_variable = 500;
#define High_threshold1 500 //High threshold for starting reading signal from peaks
#define Low_threshold1 400 //Low threshold for stopping reading signal from peaks
#define High_threshold2 1000 //High threshold for starting reading signal from peaks
#define Low_threshold2 700 //Low threshold for stopping reading signal from peaks
#define pin_input1 A2 //input signal from PD1 (D2 and 935nm)
#define pin_input2 A4 //input signal from PD2 (795nm)
#define arraysize 2000 //size of the array for storing the data of the peaks
#define dpin_out 6//digital output for triggering function generator
#define dpin_in 2 // Note: Ensure this pin supports interrupts on your board
#define alpha1_ref 0.6
#define alpha2_ref 0.50
#define pin_output1 DAC0 //feedback signal to 935nm ECDL piezo
#define pin_output2 DAC1 //feedback signal to 795nm DBR current

// --- TRIGGER LOGIC FROM TwoTriggerRead.ino ---
volatile bool trigger_active = false; 
// ----------------------------------------------

unsigned long t01, t02, t1, t2, start_time, end_time, tstartsweep, time_peak, timenow, period = 50;
int signalarray[arraysize], dsignalarray[arraysize] = {};
int i, len, counter, Range, j;
double value1 , value2;
float alpha1, alpha2, error1, error2, totalT, control_output1, control_output2;
bool trigger, flag = HIGH;
bool indicator2 = LOW; 
bool lock;

int sign2 = -1; 
volatile float laser2_K_i = 1;
volatile float laser2_K_p = 0.7;
float laser2_error_signal_current;
float laser2_error_signal_prev;
float delta_laser2 = 0;
float laser2_control_signal = 0;

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogReadResolution(12);
  
  // --- EXTERNAL TRIGGER SETUP ---
  pinMode(dpin_in, INPUT);
  attachInterrupt(digitalPinToInterrupt(dpin_in), triggerISR, CHANGE);
  trigger_active = digitalRead(dpin_in); 
  // -------------------------------

  pinMode(dpin_out, OUTPUT);
  digitalWrite(dpin_out, LOW);
  Range = (pow(2, 12) - 1) - 200;
}

void loop() {
  // Use the interrupt-updated variable
  lock = trigger_active;

  if(lock){
    start_time = micros();
    digitalWrite(dpin_out, HIGH);
    trigger = HIGH;
    indicator2 = LOW;
    tstartsweep = millis();
    counter = 0;
    
    do {
      value1 = analogRead(pin_input1);
      value2 = analogRead(pin_input2);
      timenow = millis();
      
      if (timenow - tstartsweep > period) { 
        trigger = LOW;
      }
      
      if (value1 > High_threshold1 ) {
        time_peak = micros();
        i = 0;
        do {
          signalarray[i] = value1;
          value1 = analogRead(pin_input1);
          i++;
        } while(value1 > Low_threshold1); 
        end_time = micros();
        len = i;
        if (counter == 0) {
          t01 = time_peak - start_time + peakfinder(len, end_time - time_peak);
        }
        if (counter == 2) {
          t02 = time_peak - start_time + peakfinder(len, end_time - time_peak);
        }
        counter++;
      }
      
      if (counter == 1) {
        if (value2 > High_threshold2 && indicator2 == LOW) { 
          indicator2 = HIGH;
          time_peak = micros();
          i = 0;
          do {
            signalarray[i] = value2;
            value2 = analogRead(pin_input2);
            i++;
          } while (value2 > Low_threshold2);
          end_time = micros();
          len = i;
          t2 = time_peak - start_time + peakfinder(len, end_time - time_peak);
          counter++;
        }
      }
    } while (trigger == HIGH);

    error2 = (double)t2 - t01;
    totalT = (double)t02 - t01;

    if (totalT > 0 && totalT < 160000 && error2 < totalT) {
      alpha2 = error2 / totalT;
      if (indicator2 == LOW || counter < 2) { 
        laser2_control_signal = 0;
        control_output2 = (double)650 + Range / 2.0 * laser2_control_signal;
        analogWrite(DAC0, control_output2);
        Serial.println("No third peak");
      }
      else { 
        laser2_error_signal_current =  alpha2_ref - (double)error2 / totalT;
        Serial.println(laser2_error_signal_current);
        delta_laser2 = sign2 * laser2_K_p * (laser2_error_signal_current - laser2_error_signal_prev) + sign2 * (laser2_K_i * laser2_error_signal_current);
        laser2_control_signal += delta_laser2;
        laser2_error_signal_prev = laser2_error_signal_current;
        control_output2 = (double)2072.5 + Range / 2.0 * laser2_control_signal;
        analogWrite(DAC1, control_output2);
      }
    }
    digitalWrite(dpin_out, LOW);
  }
  else {
    Serial.println("Lock disengaged");
    control_output2 = (double)2072.5;
    analogWrite(DAC1, control_output2);
  }
}

// --- ISR EXACTLY AS IN TwoTriggerRead.ino ---
void triggerISR() {
  trigger_active = digitalRead(dpin_in);
}

unsigned long peakfinder(int number, unsigned long duration) {
  unsigned long dt, peaktime;
  flag = HIGH;
  dt = duration / number;
  for (int j = 6; j < (number - 7); j++) {
    dsignalarray[j] = int((6 * signalarray[j + 6] + 5 * signalarray[j + 5] + 4 * signalarray[j + 4] + 3 * signalarray[j + 3] + 2 * signalarray[j + 2] + signalarray[j + 1] - signalarray[j - 1] - 2 * signalarray[j - 2] - 3 * signalarray[j - 3] - 4 * signalarray[j - 4] - 5 * signalarray[j - 5] - 6 * signalarray[j - 6]));
    if (dsignalarray[j] <= 0 && flag) {
      flag = LOW;
      peaktime = (j + dsignalarray[j] / (dsignalarray[j - 1] - dsignalarray[j])) * dt;
      return peaktime;
    }
  }
}
