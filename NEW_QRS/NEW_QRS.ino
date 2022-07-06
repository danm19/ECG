#include <SPI.h>
#include "SD.h"
#include "ResponsiveAnalogRead.h"

#define SCK  18
#define MISO  19
#define MOSI  23
#define CS  5

/* Pam-Tompkins algorithm variables 
 * M - Define the size for the Highpass filter 
 * N - Define the size for the Lowpass filter
 * WINSIZE - Defines the windowsize to set the sensitivty of the QRS-detection. 
 * OBS: WINSIZE can be set empirically
*/
#define M            5    
#define N           30 
#define WINSIZE    250    
#define N_SAMPLES  1000
#define MAXSIZE      6
#define next(elem)   ((elem+1)%MAXSIZE)
#define HP_CONSTANT  ((float) 1 / (float) M)
#define MAX_BPM    100
#define MAX_BPM_SIZE     6

// resolution of RNG
#define RAND_RES 100000000

const int ECG_PIN =  34;       // the number of the ECG pin (analog)
const int LEADS_OFF_PLUS_PIN = 35;
const int LEADS_OFF_MINUS_PIN = 32;
const int SD_CS = 0;

unsigned long bpm_buff[MAX_BPM_SIZE] = {0};
int bpm_wr_ptr = 0;
int bpm_rd_ptr = 0;
float bpm = 0;

float ecg_raw_data[MAXSIZE] = {0};
int ecg_wr_ptr = 0 ;
int ecg_rd_ptr = 0 ;

float hp_data[N+1] = {0};
int hp_wr_ptr = 0 ;
int hp_rd_ptr = 0 ;

float hp_sum = 0;
float lp_sum = 0;
float next_ptr = 0;

float hp_data1 = 0;
float hp_data2 = 0;

float treshold=0 ;
boolean triggered=false ;
int trig_time = 0 ;
float win_max = 0 ;
int win_idx = 0 ;

int tmp = 0;

int cprTimeRead_1 = 0 ;
int cprTimeRead_2 = 0 ;
int timeCPR = 0 ;
float cprsum;
int QRS = 0;

unsigned long previousMicros  = 0;        // will store last time LED was updated
unsigned long foundTimeMicros = 0;        // time at which last QRS was found
unsigned long old_foundTimeMicros = 0;        // time at which QRS before last was found
unsigned long currentMicros   = 0;        // current time
const long PERIOD = 1000000 / WINSIZE;

bool Plotting = true;
int i = 0;
int n_interat = 0;
int interval = 0;

const char* Filename = "/Output.csv";

void writeFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
       Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
       Serial.println("File written");
    } else {
       Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
       Serial.println("Failed to open file for appending");
       return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

boolean detect(float ecg_data){
  
  int aux = 0;
  //Put a new data in the circular buffer before the bandpass filter. 
  ecg_raw_data[ecg_wr_ptr] = ecg_data;
  ecg_wr_ptr = (ecg_wr_ptr+1)%(M+1);

  //Serial.println(ecg_raw_data[ecg_rd_ptr]);
  
  // Put a new data into the circular buffer of the High Pass filter. 
  // This step checks if the circular buffer is available for a new data. 
  if(n_interat < M)
  {
    hp_sum += ecg_raw_data[ecg_rd_ptr];
    hp_data[hp_wr_ptr] = 0; // ALTERAR
  }
  else
  {
    hp_sum += ecg_raw_data[ecg_rd_ptr];   
    aux = ecg_rd_ptr - M;
    
    if(aux < 0)
      aux += (M + 1);
    
    hp_sum -= ecg_raw_data[aux];
    aux = (ecg_rd_ptr - ((M+1)/2));
    
    if(aux < 0)
      aux += M + 1;
    
    hp_data1 = ecg_raw_data[aux];   
    hp_data2 = HP_CONSTANT * hp_sum;
    
    hp_data[hp_wr_ptr] = hp_data1 - hp_data2;      
  } 
  ecg_rd_ptr = (ecg_rd_ptr+1)%(M+1);
  hp_wr_ptr = (hp_wr_ptr+1)%(N+1);

 //Low pass filtering 
  // shift in new sample from high pass filter
  lp_sum += hp_data[hp_rd_ptr] * hp_data[hp_rd_ptr];

  if(n_interat < N){
    // first fill buffer with enough points for LP filter
    next_ptr = 0;
  }
  else{
    // shift out oldest data point
    aux = hp_rd_ptr - N;
    if(aux < 0) 
      aux += (N+1);
    
    lp_sum -= hp_data[aux] * hp_data[aux];

    next_ptr = lp_sum;
    
  } 
  // done reading HP buffer, increment position
  hp_rd_ptr = (hp_rd_ptr+1)%(N+1);
  //Serial.println(hp_data[hp_rd_ptr]);
  
  /// Adapative thresholding beat detection
  // set initial threshold        
  if(n_interat < WINSIZE) {
    if(next_ptr > treshold) {
      treshold = next_ptr;
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
  //Serial.println(next_ptr);
  // find if we have a new max
  if(next_ptr > win_max) 
    win_max = next_ptr;
  
  // find if we are above adaptive threshold
  if(next_ptr > treshold && !triggered) {
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
    //Serial.println(treshold); 
    // reset current window index
    win_idx = 0;
    win_max = -10000000;
  }
      
        // return false if we didn't detect a new QRS
  return false;

}

void setup() {
  // set the digital pin as output:
  Serial.begin(115200);
  
  pinMode(ECG_PIN, INPUT);
  pinMode(LEADS_OFF_PLUS_PIN, INPUT); // Setup for leads off detection LO +
  pinMode(LEADS_OFF_MINUS_PIN, INPUT); // Setup for leads off detection LO -
    
  SPIClass spi = SPIClass(VSPI);
  spi.begin(SCK, MISO, MOSI, CS);

  if (!SD.begin(CS)){
    Serial.println("Card Mount Failed");
    return;
  }
  else {  
    Serial.println("Card Mount Success");
  }  
  if(SD.exists(Filename))
    SD.remove(Filename);
    
  writeFile(SD,Filename,"Counter,Time,QRS");
}

void loop() {
  delay(10);
  String aux = "";
  currentMicros = micros();
  previousMicros = currentMicros;

  int next_ecg_pt = analogRead(ECG_PIN);
  boolean QRS_detected = false;

   if (currentMicros - previousMicros >= PERIOD) {
      // save the last time you blinked the LED
      previousMicros = currentMicros;
   }

  //give next data point to algorithm
  QRS_detected = detect(next_ecg_pt);
  Serial.println(QRS_detected);
  
   if(i < N_SAMPLES){
    aux += String(i);
    aux += ",";
    aux += String(currentMicros);
    aux += ",";
    aux += String(QRS_detected);
    aux += ",";
    aux += bpm_buff[bpm_rd_ptr];
    aux += "\r\n";
    appendFile(SD, Filename, aux.c_str());
    delay(20);
  }
  
  if(QRS_detected == true){
    bpm_buff[bpm_wr_ptr] = (60.0 / (((float) (foundTimeMicros - old_foundTimeMicros)) / 1000000.0));
    
    bpm_wr_ptr = (bpm_wr_ptr+1)%MAX_BPM_SIZE;
    
    bpm += bpm_buff[bpm_rd_ptr];
    
    tmp = bpm_rd_ptr - MAX_BPM_SIZE + 1;
    if(tmp < 0) 
      tmp += MAX_BPM_SIZE;
    
    bpm -= bpm_buff[tmp];
    
    bpm_rd_ptr++;
    bpm_rd_ptr %= MAX_BPM_SIZE;

    old_foundTimeMicros = foundTimeMicros;      
  }   
  i++;
}  
