#include "sensors.h"
#include "system.h"
#include "gui.h"


// ----------------------------------------------------------------------
//   Temperature sensing (onewire DS18B20)
// ----------------------------------------------------------------------

// Temperature sensor
#include <DallasTemperature.h>
#define ONE_WIRE_BUS TSAPIN
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int temp_resolution = 12;
//int temp_measureperiod = 5000;  // millis
int temp_convertdelay = 0;  // millis (calculated in setup from resolution)
int temp_idle;
unsigned long temp_lastconvertrequest = 0;
unsigned long temp_lastupdate = 0;
DeviceAddress temp0_DeviceAddress;
float temp0 = -1000.0;
float temp_setpoint = 41.0;

void setup_tempsens(void)
{
  Serial.println("Calling begin on DallasTemperature sensor");
  sensors.begin();

  int available = sensors.getDeviceCount();
  Serial.print("Sensors available: ");  Serial.println(available, DEC);

  sensors.getAddress(temp0_DeviceAddress, 0);
  temp_convertdelay = 750 / (1 << (12 - temp_resolution));
  sensors.setResolution(temp0_DeviceAddress, temp_resolution);
 
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  temp_lastconvertrequest = millis();
  temp_idle = 0;  // conversion in progress
}

void loop_tempsens(void)
{
  float ttemp; 
  
  // Start new conversion if required
  if( temp_idle == 1 ) {
    if( millis() - temp_lastconvertrequest >= temp_measureperiod ) {
      //Log("Requesting temperature...");
      sensors.requestTemperatures();
      temp_lastconvertrequest = millis();
      temp_idle = 0;  // conversion in progress
    }
    return;
  }

  // Handle ready temperature measurement
  ttemp = sensors.getTempCByIndex(0);  // first device on bus
  if( ttemp>0 && ttemp<70 ) temp0 = ttemp;
  else btLog("Ignoring invalid temp0=" + String(ttemp));
 
  temp_idle = 1;  //  ready for next
}


// ----------------------------------------------------------------------
//   Temperature sensing (NTC 10k)
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
float NTCtemp(int i) { return(i-25); }  // temp corresponding to each index
int NTCRk_len;
int NTCni = 0;
float temp1 = -1000.0;

void setup_tempsens1(void)
{
  NTCni = 45;  // initial search index
  temp1 = -1000.0;
  NTCRk_len = sizeof(NTCRk)/sizeof(float);
}

void loop_tempsens1(void) 
{
  float ttemp;
  int i;
  float Rkmeas;
  int aread;
  float tvread;

  if( millis()-temp1_lastupdate<temp_measureperiod ) return;
  
  // Measure ADC voltage and calculate thermistor resistance
  const float Rtseries = 14930;  // measured
  const float tvref = 5.0;
  aread = analogRead(TSA2PIN);
  tvread = aread/4095.0*3.3;  // voltage read
  Rkmeas = (1/1000.0)*(tvread/tvref)*Rtseries/(1 - (tvread/tvref));

  // Try locate index i such that NTCRk[i]>=Rmeas>NTCRk[i+1]
  i = NTCni;  // initialise from last
  while( i<NTCRk_len-2 && NTCRk[i+1]>=Rkmeas ) i++;
  while( i>0 && NTCRk[i]<Rkmeas ) i--;
  NTCni = i;  // store for next
 
  // Convert to temperature (interpolate)
  if( NTCRk[i]>=Rkmeas && NTCRk[i+1]<Rkmeas ) {
    ttemp = NTCtemp(i) + (Rkmeas - NTCRk[i])/(NTCRk[i+1]-NTCRk[i])*(NTCtemp(i+1)-NTCtemp(i));
  } else {
    ttemp = -1000;
  }

  temp1 = ttemp;
  temp1_lastupdate = millis();
  //Serial.println("loop_tempsens1: voltage=" + String(tvread) + ", R=" + String(Rkmeas) + ", temp1=" + String(temp1));
}


// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

volatile int flow_cnt0, flow_cnt1;
float flow_lpm0, flow_lpm1;
unsigned long flow_lastupdate = 0;
//const int flow_measureperiod = 1000;  // millis
void IRAM_ATTR flowISR0(void) {
  flow_cnt0++;
}
void IRAM_ATTR flowISR1(void) {
  flow_cnt1++;
}

float flow_thr0, flow_thr1;
int flow_lasts0, flow_lasts1;
unsigned long flow_lastch0, flow_lastch1;

void setup_flowsens(void)
{
  flow_cnt0 = 0;  flow_cnt1 = 0;
  flow_lpm0 = 0;  flow_lpm1 = 0;
  flow_lastupdate = millis();
  attachInterrupt(digitalPinToInterrupt(FSPINS[0]), flowISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(FSPINS[1]), flowISR1, RISING);

  flow_thr0 = flow_thr1 = 2.25;
  flow_lasts0 = flow_lasts1 = 0;
  flow_lastch0 = flow_lastch1 = flow_lastupdate;
}

void loop_flowsens(void)
{
  if( millis()-flow_lastupdate<flow_measureperiod ) return;

  // New measurement
  unsigned int flowsens_duration = millis() - flow_lastupdate;
  float flfreq = (float)flow_cnt0 / flowsens_duration * 1000.0; // hertz
  flow_lpm0 = 10.0 / 82 * flfreq;
  flfreq = (float)flow_cnt1 / flowsens_duration * 1000.0;
  flow_lpm1 = 10.0 / 82 * flfreq; // 10lpm==82Hz?
  flow_cnt0 = 0;  flow_cnt1 = 0;
  flow_lastupdate = millis();
  //("FS lpm=" + String(flow_lpm0) + "," + String(flow_lpm1));

  // Hysteresis for new measurement
  if( ( flow_lasts0==0 && flow_lpm0>=flow_thr0 ) || ( flow_lasts0==1 && flow_lpm0<flow_thr0 ) ) {
    flow_lasts0 = flow_lpm0>=flow_thr0;
    flow_lastch0 = flow_lastupdate;
  }
  if( ( flow_lasts1==0 && flow_lpm1>=flow_thr1 ) || ( flow_lasts1==1 && flow_lpm1<flow_thr1 ) ) {
    flow_lasts1 = flow_lpm1>=flow_thr1;
    flow_lastch1 = flow_lastupdate;
  }
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
  float MAXkpa = 500;  // 0.5MPa sensor
  float vread = (3.3/4095)*analogRead(PSPIN);  // ADC voltage
  float sensv = 5/3.3*vread;  // inferred sensor voltage
  
  float kpa = MAXkpa*(sensv - 0.5)/(4.5-0.5);
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