#include "sensors.h"
#include "system.h"
#include "gui.h"
#include <Preferences.h>


// ----------------------------------------------------------------------
//   Level sensing
// ----------------------------------------------------------------------

bool level_high0, level_high1;
bool level_chigh0, level_chigh1;
bool tank_full = false;
bool tank_empty = false;
int level_lastlt0, level_lastlt1;  // last low time
int level_lastht0, level_lastht1;  // last high time

float level_hest;
unsigned int levelest_lastupdate;
int level_high0last, level_high1last;
float tank_innerd = 15.06;  // cm
float level_height0 = 20.4;  // cm
float level_height1 = 33.6;  // cm
float level_htank = 38.6;  // cm


void setup_levelsens(void) {
  level_high0 = level_high1 = false;
  level_chigh0 = level_chigh1 = false;
  level_lastlt0 = level_lastlt1 = level_lastht0 = level_lastht1 = 0;
  level_hest = 0;
  levelest_lastupdate = millis();
  level_high0last = level_high1last = false;
}

void update_tank_level_states(void)
{
  tank_full = level_high1; // Tank is full if the upper sensor is high
  tank_empty = !level_high0 && !level_high1; // Tank is empty if both sensors are low
}

void loop_levelsens(void)
{
  // Mechanical sensors with 300ms software debouncing
  static int raw_state0 = -1, raw_state1 = -1;
  static unsigned long raw_change_time0 = 0, raw_change_time1 = 0;

  int current_raw0 = (digitalRead(LSPINS[0]) == LSPINSlv[0]) ? 0 : 1;
  if (current_raw0 != raw_state0) {
    raw_state0 = current_raw0;
    raw_change_time0 = millis();
  } else if (millis() - raw_change_time0 >= 300) {
    if (level_high0 != raw_state0) {
      level_high0 = raw_state0;
      if (level_high0) level_lastht0 = millis();
      else level_lastlt0 = millis();
    }
  }

  int current_raw1 = (digitalRead(LSPINS[1]) == LSPINSlv[1]) ? 0 : 1;
  if (current_raw1 != raw_state1) {
    raw_state1 = current_raw1;
    raw_change_time1 = millis();
  } else if (millis() - raw_change_time1 >= 300) {
    if (level_high1 != raw_state1) {
      level_high1 = raw_state1;
      if (level_high1) level_lastht1 = millis();
      else level_lastlt1 = millis();
    }
  }

  // Capacitive sensors
  if( digitalRead(LSCPINS[0])==LSCPINSlv[0] ) level_chigh0 = 0;
  else level_chigh0 = 1;
  if( digitalRead(LSCPINS[1])==LSCPINSlv[1] ) level_chigh1 = 0;
  else level_chigh1 = 1;

  // Estimate change in level
  float levelest_updateduration = millis() - levelest_lastupdate;
  float deltavol = 1000.0*(flow_lpm1 - flow_lpm0)*(levelest_updateduration/1000.0/60.0);  //cm^3
  float tank_area = 3.141592653*powf(tank_innerd/2, 2.0);
  float deltah = deltavol/tank_area;
  level_hest += deltah;
  levelest_lastupdate = millis();

  // Level measurement resets estimate
  if( level_high0!=level_high0last ) level_hest = level_height0;
  if( level_high1!=level_high1last ) level_hest = level_height1;
  level_high0last = level_high0;
  level_high1last = level_high1;

  update_tank_level_states();
}


// ----------------------------------------------------------------------
//   Temperature sensing (onewire DS18B20)
// ----------------------------------------------------------------------

// Temperature sensor (DS18B20 OneWire - Retired to free GPIO 5 for RPBALLVALVE)
float temp0 = -1000.0;

void setup_tempsens0(void)
{
  // Retired
}

void loop_tempsens0(void)
{
  // Retired
}


// ----------------------------------------------------------------------
// Temperature sensing (NTC 10k B(25/50)=3950)
// ----------------------------------------------------------------------


unsigned long temp1_lastupdate = 0;

// The array below contains NTC resistance values for temperatures from -25 to 125 degrees celcius
// in increments of 1 degree
const float NTCRk[] = {133.5,125.67,118.35,111.5,105.08,99.077,93.447,88.175,83.23,78.591,
                       74.238,70.153,66.316,62.712,59.325,56.142,53.148,50.331,47.68,45.184,
                       42.834,40.62,38.533,36.566,34.71,32.96,31.308,29.749,28.276,26.885,
                       25.57,24.327,23.152,22.041,20.989,19.993,19.051,18.158,17.312,16.511,
                       15.751,15.031,14.347,13.699,13.083,12.499,11.944,11.417,10.916,10.44,
                       10,9.5569,9.1474,8.7578,8.3869,8.0338,7.6975,7.3772,7.072,6.7811,
                       6.5038,6.2393,5.9871,5.7465,5.5168,5.2976,5.0883,4.8884,4.6974,4.515,
                       4.3406,4.1739,4.0145,3.8621,3.7162,3.588,3.4431,3.3153,3.1929,3.0756,
                       2.9633,2.8557,2.7526,2.6537,2.5589,2.468,2.3808,2.2971,2.2169,2.1398,
                       2.0658,1.9948,1.9266,1.8611,1.7981,1.7376,1.6794,1.6235,1.5697,1.518,
                       1.473,1.4204,1.3744,1.3301,1.2874,1.2463,1.2067,1.1686,1.1319,1.0965,
                       1.0625,1.0296,0.99792,0.96738,0.93792,0.9095,0.88208,0.85563,0.8301,0.80546,
                       0.78167,0.7587,0.73652,0.71509,0.6944,0.6744,0.65507,0.6364,0.6184,0.60089,
                       0.58401,0.56769,0.5519,0.53663,0.52185,0.50755,0.49371,0.48032,0.46735,0.45479,
                       0.44263,0.43085,0.41944,0.40838,0.39767,0.38729,0.37723,0.36748,0.35803,0.34886,
                       0.33997};

float NTCtemp(int i) { return(i-25); } // temp corresponding to each index
int NTCRk_len;
int NTCni = 0;
float temp1 = -1000.0;

#define TEMP1_MEDIAN_WINDOW 5
float temp1_buf[TEMP1_MEDIAN_WINDOW];
int temp1_buf_count = 0;
int temp1_buf_idx = 0;

float get_tempsens1()
{
  float ttemp = -1000;  // invalid

  // Measure ADC voltage and calculate thermistor resistance
  const float Rtkseries = 5.00; // measured precisely as 5.00k
  
  // Take 9 ADC readings with 500us spacing to reject electrical noise spikes
  int samples[9];
  for (int j = 0; j < 9; j++) {
    samples[j] = analogRead(TSA2PIN);
    if (j < 8) delayMicroseconds(500);
  }

  // Insertion sort to find median ADC reading
  for (int j = 1; j < 9; j++) {
    int key = samples[j];
    int k = j - 1;
    while (k >= 0 && samples[k] > key) {
      samples[k + 1] = samples[k];
      k--;
    }
    samples[k + 1] = key;
  }

  int median_adc = samples[4];
  if (median_adc >= 4094 || median_adc <= 10) return -1000.0; // Prevent divide by zero if sensor open/shorted

  // Ratiometric calculation (voltage cancels out)
  float Rkmeas = (Rtkseries * (float)median_adc) / (4095.0f - (float)median_adc);

  // Try locate index i such that NTCRk[i]>=Rmeas>NTCRk[i+1]
  int i = NTCni; // initialise from last
  while( i<NTCRk_len-2 && NTCRk[i+1]>=Rkmeas ) i++;
  while( i>0 && NTCRk[i]<Rkmeas ) i--;
  NTCni = i; // store for next
  
  // Convert to temperature (interpolate)
  if( Rkmeas<=NTCRk[0] && Rkmeas>=NTCRk[NTCRk_len - 1] ) {
    float R1 = NTCRk[i];
    float R2 = NTCRk[i+1];
    float T1 = NTCtemp(i);
    float T2 = NTCtemp(i+1);
    
    ttemp = T1 + (Rkmeas - R1) * (T2 - T1) / (R2 - R1);
  }

  return(ttemp);
}

void setup_tempsens1(void)
{
  NTCni = 45; // initial search index
  temp1 = -1000.0;
  NTCRk_len = sizeof(NTCRk)/sizeof(float);
  temp1_buf_count = 0;
  temp1_buf_idx = 0;
}


void loop_tempsens1(void)
{
  if( millis()-temp1_lastupdate<temp_measureperiod ) return;
  temp1_lastupdate = millis();
  
  float ttemp = get_tempsens1();

  // If reading not obviously bad (valid range 0°C to 100°C)
  if( ttemp > 0.0f && ttemp < 100.0f ) {
    temp1_buf[temp1_buf_idx] = ttemp;
    temp1_buf_idx = (temp1_buf_idx + 1) % TEMP1_MEDIAN_WINDOW;
    if( temp1_buf_count < TEMP1_MEDIAN_WINDOW ) temp1_buf_count++;

    // Calculate median of rolling window
    float sorted_buf[TEMP1_MEDIAN_WINDOW];
    for (int k = 0; k < temp1_buf_count; k++) sorted_buf[k] = temp1_buf[k];
    for (int j = 1; j < temp1_buf_count; j++) {
      float key = sorted_buf[j];
      int k = j - 1;
      while (k >= 0 && sorted_buf[k] > key) {
        sorted_buf[k + 1] = sorted_buf[k];
        k--;
      }
      sorted_buf[k + 1] = key;
    }

    temp1 = sorted_buf[temp1_buf_count / 2];
  }
} 


// ----------------------------------------------------------------------
// Temperature sensing (all)
// ----------------------------------------------------------------------

float temp = -1000.0;

void setup_tempsens(void)
{
  //setup_tempsens0(); // DS18B20 OneWire retired
  setup_tempsens1();
}

void loop_tempsens(void)
{
  //loop_tempsens0(); // DS18B20 OneWire retired
  loop_tempsens1();

  temp = temp1;
}


// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

volatile int flow_cnt0, flow_cnt1;
volatile unsigned long total_flow_pulses0 = 0, total_flow_pulses1 = 0;
float flow_rec_scale = 1.0f; // Multiplier to convert raw recovery flow to delivery flow units
float flow_lpm0 = 0.0f, flow_lpm1 = 0.0f, flow_lpm1_est = 0.0f;
unsigned long flow_lastupdate = 0;
volatile unsigned long last_flow_time0 = 0;
volatile unsigned long last_flow_time1 = 0;
//const int flow_measureperiod = 1000;  // millis
void IRAM_ATTR flowISR0(void) {
  unsigned long now = micros();
  if (now - last_flow_time0 >= 1000) {
    flow_cnt0 = flow_cnt0 + 1;
    total_flow_pulses0 = total_flow_pulses0 + 1;
    last_flow_time0 = now;
  }
}
void IRAM_ATTR flowISR1(void) {
  unsigned long now = micros();
  if (now - last_flow_time1 >= 1000) {
    flow_cnt1 = flow_cnt1 + 1;
    total_flow_pulses1 = total_flow_pulses1 + 1;
    last_flow_time1 = now;
  }
}

void flow_reset_total_pulses(void) {
  total_flow_pulses0 = 0;
  total_flow_pulses1 = 0;
}

float flow_thr0, flow_thr1;
//int flow_lasts0, flow_lasts1;
int flow_lastlt0, flow_lastlt1;  // last low time
int flow_lastht0, flow_lastht1;  // last high time
//unsigned long flow_lastch0, flow_lastch1;

int calibf_table_size = 0;
float calibf_table_raw[CALIBF_MAX_TABLE_PTS];
float calibf_table_corr[CALIBF_MAX_TABLE_PTS];

float get_corrected_recovery_flow(float raw_rec_lpm) {
  if (raw_rec_lpm <= 0.05f) return 0.0f;
  if (calibf_table_size < 2) {
    // Fallback to single scale factor if no full curve table
    return raw_rec_lpm * flow_rec_scale;
  }

  // Below lowest calibrated point: scale linearly using lowest point ratio
  if (raw_rec_lpm <= calibf_table_raw[0]) {
    float slope0 = calibf_table_corr[0] / calibf_table_raw[0];
    return raw_rec_lpm * slope0;
  }

  // Above highest calibrated point: extrapolate using the slope of the final segment
  if (raw_rec_lpm >= calibf_table_raw[calibf_table_size - 1]) {
    int last = calibf_table_size - 1;
    float slope_last = (calibf_table_corr[last] - calibf_table_corr[last - 1]) / 
                       (calibf_table_raw[last] - calibf_table_raw[last - 1]);
    return calibf_table_corr[last] + slope_last * (raw_rec_lpm - calibf_table_raw[last]);
  }

  // Piecewise linear interpolation between points
  for (int i = 0; i < calibf_table_size - 1; i++) {
    if (raw_rec_lpm >= calibf_table_raw[i] && raw_rec_lpm <= calibf_table_raw[i + 1]) {
      float frac = (raw_rec_lpm - calibf_table_raw[i]) / (calibf_table_raw[i + 1] - calibf_table_raw[i]);
      return calibf_table_corr[i] + frac * (calibf_table_corr[i + 1] - calibf_table_corr[i]);
    }
  }

  return raw_rec_lpm * flow_rec_scale;
}

void save_calibf_table(int n_pts, const float raw_pts[], const float corr_pts[]) {
  if (n_pts < 2 || n_pts > CALIBF_MAX_TABLE_PTS) return;

  Preferences p;
  p.begin("calibf", false);
  p.putInt("n_pts", n_pts);
  for (int i = 0; i < n_pts; i++) {
    p.putFloat(("r_" + String(i)).c_str(), raw_pts[i]);
    p.putFloat(("c_" + String(i)).c_str(), corr_pts[i]);
  }
  // Store aggregate K for fallback
  if (raw_pts[n_pts - 1] > 0) {
    float sum_raw = 0, sum_corr = 0;
    for (int i = 0; i < n_pts; i++) { sum_raw += raw_pts[i]; sum_corr += corr_pts[i]; }
    float avg_k = (sum_raw > 0) ? (sum_corr / sum_raw) : 0.7477f;
    p.putFloat("k_rec", avg_k);
    flow_rec_scale = avg_k;
  }
  p.end();

  calibf_table_size = n_pts;
  for (int i = 0; i < n_pts; i++) {
    calibf_table_raw[i] = raw_pts[i];
    calibf_table_corr[i] = corr_pts[i];
  }
  Serial.printf("Sensors: Saved %d-point calibration curve to NVRAM.\n", n_pts);
}

void load_calibf_table(void) {
  Preferences p;
  p.begin("calibf", true);
  if (p.isKey("n_pts")) {
    int n = p.getInt("n_pts", 0);
    if (n >= 2 && n <= CALIBF_MAX_TABLE_PTS) {
      calibf_table_size = n;
      for (int i = 0; i < n; i++) {
        calibf_table_raw[i] = p.getFloat(("r_" + String(i)).c_str(), 0.0f);
        calibf_table_corr[i] = p.getFloat(("c_" + String(i)).c_str(), 0.0f);
      }
      Serial.printf("Sensors: Loaded %d-point calibration curve from NVRAM.\n", calibf_table_size);
      for (int i = 0; i < calibf_table_size; i++) {
        Serial.printf("  Point %d: Raw=%.2f LPM -> Corrected=%.2f LPM (K=%.4f)\n", 
                      i + 1, calibf_table_raw[i], calibf_table_corr[i], 
                      calibf_table_raw[i] > 0 ? (calibf_table_corr[i] / calibf_table_raw[i]) : 1.0f);
      }
    }
  }
  if (p.isKey("k_rec")) {
    flow_rec_scale = p.getFloat("k_rec", 0.7477f);
    Serial.printf("Sensors: Base K_rec = %.4f\n", flow_rec_scale);
  }
  p.end();
}

void setup_flowsens(void)
{
  flow_cnt0 = 0;  flow_cnt1 = 0;
  total_flow_pulses0 = 0;  total_flow_pulses1 = 0;
  flow_lpm0 = 0;  flow_lpm1 = 0;
  flow_lastupdate = millis();
  attachInterrupt(digitalPinToInterrupt(FSPINS[0]), flowISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(FSPINS[1]), flowISR1, RISING);

  load_calibf_table();

  flow_thr0 = flow_thr1 = 1.5;
  flow_lastlt0 = 0;  flow_lastlt1 = 0;
  flow_lastht0 = 0;  flow_lastht1 = 0;
}

void loop_flowsens(void)
{
  if( millis()-flow_lastupdate<flow_measureperiod ) return;

  // Measurement complete
  unsigned int flowsens_duration = millis() - flow_lastupdate;
  float flfreq0 = (float)flow_cnt0 / flowsens_duration * 1000.0f; // hertz
  flow_lpm0 = 10.0f / 82.0f * flfreq0;
  float flfreq1 = (float)flow_cnt1 / flowsens_duration * 1000.0f;
  flow_lpm1 = 10.0f / 82.0f * flfreq1; // Raw uncalibrated recovery flow (used by control loops)
  flow_lpm1_est = get_corrected_recovery_flow(flow_lpm1); // Corrected recovery flow in delivery flow units

  // Reset and start new measurement
  flow_cnt0 = 0;  flow_cnt1 = 0;
  flow_lastupdate = millis();
  //Serial.println("FS lpm=" + String(flow_lpm0) + "," + String(flow_lpm1));

  // Store last high time for flow
  if( flow_lpm0>flow_thr0 ) flow_lastht0 = millis();
  else flow_lastlt0 = millis();
  if( flow_lpm1>flow_thr1 ) flow_lastht1 = millis();
  else flow_lastlt1 = millis();

  // // Hysteresis for new measurement
  // if( ( flow_lasts0==0 && flow_lpm0>=flow_thr0 ) || ( flow_lasts0==1 && flow_lpm0<flow_thr0 ) ) {
  //   flow_lasts0 = flow_lpm0>=flow_thr0;
  //   flow_lastch0 = flow_lastupdate;
  // }
  // if( ( flow_lasts1==0 && flow_lpm1>=flow_thr1 ) || ( flow_lasts1==1 && flow_lpm1<flow_thr1 ) ) {
  //   flow_lasts1 = flow_lpm1>=flow_thr1;
  //   flow_lastch1 = flow_lastupdate;
  // }
}


// ----------------------------------------------------------------------
//   Pressure sensing
// ----------------------------------------------------------------------

float psens0 = -1000.0;
unsigned long psens_lastupdate = 0;

// The cheap pressure sensors seem to use a 5V supply and (dodgy) datasheets 
// show linear between measured (0.5-4.5V) to (0-MAXkPa).  Resistor divider 
// fitted into cable to reduce range to 0-3.3V)
float psens_readkpa(void)
{
  float kpamax = 500;  // 0.5MPa sensor
  float vread = (3.3/4095)*analogRead(PSPIN);  // ADC voltage
  float sensv = 5/3.3*vread;  // inferred sensor voltage
  
  float kpa = kpamax*(sensv - 0.45)/(4.5-0.5);
  return kpa;
}

void setup_psens(void)
{
  psens0 = psens_readkpa();
}

void loop_psens(void)
{
  if( millis()-psens_lastupdate<psens_measureperiod ) return;
  psens_lastupdate = millis();

  psens0 = psens_readkpa();
}


// ----------------------------------------------------------------------
//   Level (range) sensing
// ----------------------------------------------------------------------

//#include <SoftwareSerial.h>

//SoftwareSerial mySerial(27,14); // RX, TX (green 27, blue 14, on unit installed on tank --- 32 blue, 31 green as currently wired)
//SoftwareSerial rlSerial(32,33); // RX, TX (green 27, blue 14, on unit installed on tank --- 32 blue, 31 green as currently wired)
HardwareSerial rlSerial(1);

unsigned char rldata[4] = {};
float rldistance;
int rlevdsens;

void setup_rlevsens(void)
{
  //rlSerial.begin(9600); 
  rlSerial.begin(9600, SERIAL_8N1, RLEVRXPIN, RLEVTXPIN);
}

void loop_rlevsens(void)
{
  rldata[0] = 0;
  if( rlSerial.available()>3 && rlSerial.peek()==0xff) {
    for( int i=0; i<4; i++ ) rldata[i] = rlSerial.read();
    rlSerial.flush();
  } else if( rlSerial.available()>3 && rlSerial.peek()!=0xff) rlSerial.read();

  if( rldata[0]==0xff ) {
    int sum = (rldata[0]+rldata[1]+rldata[2]) & 0x00FF;
    if( sum==rldata[3] ) {
      rldistance = (rldata[1]<<8)+rldata[2];
      rlevdsens = rldistance;  // what to do here?

      //Serial.println("distance=" + String(rldistance) + "mm");
    } //else Serial.println("loop_rlevel: checksum error");
  }
}