#include "sensors.h"
#include "system.h"
#include "gui.h"


// ----------------------------------------------------------------------
//   Temperature sensing
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
DeviceAddress temp0_DeviceAddress, temp1_DeviceAddress;
float temp0 = -1000.0, temp1 = -1000;

void setup_tempsens(void)
{
  Serial.println("Calling begin on DallasTemperature sensor");
  sensors.begin();

  int available = sensors.getDeviceCount();
  Serial.print("Sensors available: ");  Serial.println(available, DEC);

  sensors.getAddress(temp0_DeviceAddress, 0);
  sensors.getAddress(temp1_DeviceAddress, 1);
  temp_convertdelay = 750 / (1 << (12 - temp_resolution));
  sensors.setResolution(temp0_DeviceAddress, temp_resolution);
  sensors.setResolution(temp1_DeviceAddress, temp_resolution);

  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  temp_lastconvertrequest = millis();
  temp_idle = 0;  // conversion in progress
}

void loop_tempsens(void)
{
  float ttemp; 
  
  // Start new conversion if required
  if ( temp_idle == 1 ) {
    if ( millis() - temp_lastconvertrequest >= temp_measureperiod ) {
      //Log("Requesting temperatures...");
      sensors.requestTemperatures();
      temp_lastconvertrequest = millis();
      temp_idle = 0;  // conversion in progress
    }
    return;
  }

  // Handle ready temperature measurement
  ttemp = sensors.getTempCByIndex(0);  // first device on bus
  if( ttemp>0 && ttemp<60 ) temp0 = ttemp;
  else btLog("Ignoring invalid temp0=" + String(ttemp));
  ttemp = sensors.getTempCByIndex(1);
  if( ttemp>0 && ttemp<60 ) temp1 = ttemp;
  else btLog("Ignoring invalid temp1=" + String(ttemp));

  temp_idle = 1;  //  ready for next
}




// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

volatile int flow_cnt0, flow_cnt1;
float flow_lpm0, flow_lpm1;
unsigned long flow_lastupdate = 0;
//const int flow_measureperiod = 1000;  // millis
void flowISR0(void) {
  flow_cnt0++;
}
void flowISR1(void) {
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
  if ( ( flow_lasts0==0 && flow_lpm0>=flow_thr0 ) || ( flow_lasts0==1 && flow_lpm0<flow_thr0 ) ) {
    flow_lasts0 = flow_lpm0>=flow_thr0;
    flow_lastch0 = flow_lastupdate;
  }
  if ( ( flow_lasts1==0 && flow_lpm1>=flow_thr1 ) || ( flow_lasts1==1 && flow_lpm1<flow_thr1 ) ) {
    flow_lasts1 = flow_lpm1>=flow_thr1;
    flow_lastch1 = flow_lastupdate;
  }
}
