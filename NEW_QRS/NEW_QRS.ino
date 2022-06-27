#include <stdio.h>
#include <stdlib.h>

#define M            5
#define N           30
#define WINSIZE    250

#define MAXSIZE 6
#define next(elem) ((elem+1)%MAXSIZE)
#define HP_CONSTANT   ((float) 1 / (float) M)
#define MAX_BPM     100

// resolution of RNG
#define RAND_RES 100000000

const int ECG_PIN =  34;       // the number of the ECG pin (analog)
const int LEADS_OFF_PLUS_PIN = 35;
const int LEADS_OFF_MINUS_PIN = 32;

// Declaration of the variables about ECG data collection.
// A buffer circular will be used to save data from ECG signal to High-Pass and Low-Pass filter. 
float ecg_raw_data[M + 1] = {0}; // Size buffer equal a M+1
int ecg_init_ptr = 0; // Initial index of the circular buffer.
int ecg_final_ptr = 0; // Final index of the circular buffer.

// Declaration of the variables about High-Pass Filter. 
float hp_buff[N + 1] = {0};
float hp_sum = 0;
float hp_data1 = 0;
float hp_data2 = 0;
int hp_buff_WR_idx = 0;
int hp_buff_RD_idx = 0;

// Declaration of the variables about Low-Pass Filter. 
float lp_sum = 0;

boolean QRS_detected = false;
int n_interat = 0;
int aux = 0;

// working variables for adaptive thresholding
float treshold = 0;
float win_max = 0;
float next_eval_pt = 0;
int trig_time = 0;
int win_idx = 0;
boolean triggered = false;

boolean detect(float ecg_data)
{
  ecg_raw_data[ecg_init_ptr] = ecg_data;
  next(ecg_init_ptr);

  if(n_interat < M)
  {
    hp_sum += ecg_raw_data[ecg_final_ptr];
    hp_buff[hp_buff_WR_idx] = 0; // ALTERAR
  }

  else
  {
    hp_sum += ecg_raw_data[ecg_final_ptr];   
    aux = ecg_final_ptr - M;
    
    if(aux < 0)
      aux += (M + 1);
    
    hp_sum -= ecg_raw_data[aux];
    aux = (ecg_final_ptr - ((M+1)/2));
    
    if(aux < 0)
      aux += M + 1;
    
    hp_data1 = ecg_raw_data[aux];   
    hp_data2 = HP_CONSTANT * hp_sum;
    
    hp_buff[hp_buff_WR_idx] = hp_data1 - hp_data2;
  }
    if((ecg_final_ptr+1) < M+1)
        ecg_final_ptr = ecg_final_ptr;
    else
        ecg_final_ptr = 0;

    if((ecg_final_ptr+1) < M+1)
        ecg_final_ptr = ecg_final_ptr;
    else
        ecg_final_ptr = 0;

 /* Low pass filtering */
  
  // shift in new sample from high pass filter
  lp_sum += hp_buff[ecg_final_ptr] * hp_buff[ecg_final_ptr];
  
  if(n_interat < N){
    // first fill buffer with enough points for LP filter
    next_eval_pt = 0;
    
  }
  else{
    // shift out oldest data point
    aux = hp_buff_RD_idx - N;
    if(aux < 0) aux += (N+1);
    
    lp_sum -= hp_buff[aux] * hp_buff[aux];
    
    next_eval_pt = lp_sum;
  }
  
  // done reading HP buffer, increment position
  hp_buff_RD_idx++;
  hp_buff_RD_idx %= (N+1);
  

  /* Adapative thresholding beat detection */
  // set initial threshold        
  if(n_interat < WINSIZE) {
    if(next_eval_pt > treshold) {
      treshold = next_eval_pt;
    }

                // only increment n_interat iff it is less than WINSIZE
                // if it is bigger, then the counter serves no further purpose
                n_interat++;
  }
  
  // check if detection hold off period has passed
  if(triggered == true){
    trig_time++;
    
    if(trig_time >= 100){
      triggered = false;
      trig_time = 0;
    }
  }
  
  // find if we have a new max
  if(next_eval_pt > win_max) win_max = next_eval_pt;
  
  // find if we are above adaptive threshold
  if(next_eval_pt > treshold && !triggered) {
    triggered = true;

    return true;
  }
        // else we'll finish the function before returning FALSE,
        // to potentially change threshold
          
  // adjust adaptive threshold using max of signal found 
  // in previous window            
  if(win_idx++ >= WINSIZE){
    // weighting factor for determining the contribution of
    // the current peak value to the threshold adjustment
    float gamma = 0.175;
    
    // forgetting factor - 
    // rate at which we forget old observations
                // choose a random value between 0.01 and 0.1 for this, 
    float alpha = 0.01 + ( ((float) random(0, RAND_RES) / (float) (RAND_RES)) * ((0.1 - 0.01)));
    
                // compute new threshold
    treshold = alpha * gamma * win_max + (1 - alpha) * treshold;
    
    // reset current window index
    win_idx = 0;
    win_max = -10000000;
  }
      
        // return false if we didn't detect a new QRS
  return false;
    
}
  

void setup() {
  // set the digital pin as output:
  pinMode(ECG_PIN, INPUT);

  // leads for electrodes off detection
  pinMode(LEADS_OFF_PLUS_PIN, INPUT); // Setup for leads off detection LO +
  pinMode(LEADS_OFF_MINUS_PIN, INPUT); // Setup for leads off detection LO -

  Serial.begin(115200);

}

void loop() {
  int next_ecg_pt = analogRead(ECG_PIN);
  
  delay(50);
  // give next data point to algorithm
  QRS_detected = detect(next_ecg_pt);
  

}
