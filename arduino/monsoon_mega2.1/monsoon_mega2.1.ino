//Fred Nicolls, March 2018

HardwareSerial &btSerial = Serial3;


// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

const int RPINS0[8] = {23, 25, 27, 29, 31, 33, 35, 37};  // power relay board
const int RPINS1[8] = {39, 41, 43, 45, 47, 49, 51, 53};  // valve relay board
const int LSPINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};  // level sense inputs
const int FSPINS[4] = {2, 3, 4, 5};  // flow sensor inputs (on interrupt pins)
const int TSAPIN = A13;  // onewire digital
const int RSTSENS = 6;  

const int LEVELHIGH = LOW, LEVELLOW = HIGH;
const int RELAYOFF = HIGH, RELAYON = LOW;

// Cache relay pin changes
int RPINS0v[8], RPINS1v[8];  // next values
int rpinsv_reset(void) { 
  for( int i=0; i<8; i++ ) { RPINS0v[i] = RELAYOFF;  RPINS1v[i] = RELAYOFF; }
}
int rpinsv_get(void) { 
  for( int i=0; i<8; i++ ) { 
    RPINS0v[i] = digitalRead(RPINS0[i]);  RPINS1v[i] = digitalRead(RPINS1[i]);
  }
}
int rpinsv_set(void) { 
  for( int i=0; i<8; i++ ) { 
    if( digitalRead(RPINS0[i])!=RPINS0v[i] ) digitalWrite(RPINS0[i], RPINS0v[i]);
    if( digitalRead(RPINS1[i])!=RPINS1v[i] ) digitalWrite(RPINS1[i], RPINS1v[i]);
  }
}

void pins_off()
{
  // Digital outputs for relays (high is off)
  for( int i=0; i<8; i++ ) {
    digitalWrite(RPINS0[i], RELAYOFF);  // power
    digitalWrite(RPINS1[i], RELAYOFF);  // valves
  }
  rpinsv_reset();  report_rpins();
}

void setup_pins()
{
  // Digital outputs for relays (high is off)
  for( int i=0; i<8; i++ ) {
    pinMode(RPINS0[i], OUTPUT);
    pinMode(RPINS1[i], OUTPUT);
  }
  pins_off();

  // Digital inputs for flow sensors
  for( int i=0; i<4; i++ ) pinMode(FSPINS[i], INPUT_PULLUP);
  
  // Level sensors
  for( int i=0; i<8; i++ ) pinMode(LSPINS[i], INPUT_PULLUP);

  // Temperature setup on TSAPIN handled by onewire

  // Reset sense pin
  pinMode(RSTSENS, INPUT);
}


// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

// Temperature sensor
#include <DallasTemperature.h>
#define ONE_WIRE_BUS TSAPIN
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

int temp_resolution = 12;
int temp_measureperiod = 5000;  // millis
int temp_convertdelay = 0;  // millis (calculated in setup from resolution)
int temp_idle;
unsigned long temp_lastconvertrequest = 0;
unsigned long temp_lastupdate = 0;
DeviceAddress temp0_DeviceAddress, temp1_DeviceAddress;
float temp0 = -1000.0, temp1 = -1000;
float temp_setpoint = 47.0;
float temp_reqsetpoint = temp_setpoint;

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
  // Start new conversion if required
  if( temp_idle==1 ) {
    if( millis()-temp_lastconvertrequest>=temp_measureperiod ) {
      //Log("Requesting temperatures..."); 
      sensors.requestTemperatures();
      temp_lastconvertrequest = millis();
      temp_idle = 0;  // conversion in progress
    } 
    return;
  }

  // Handle ready temperature measurement
  temp0 = sensors.getTempCByIndex(0);  // first device on bus
  if( temp0<0 ) temp0 = 1000;
  temp1 = sensors.getTempCByIndex(1);
  if( temp1<0 ) temp1 = 1000;
  temp_lastupdate = millis();
  //Log("Temperatures: " + String(temp0) + "," + String(temp1) + " (lastupdate=" + temp_lastupdate + ")");

  temp_idle = 1;  //  ready for next
} 


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int rhtrs[4] = {0, 1, 2, 3};  // heater outputs
int lshtrs[4] = {1, 1, 3, 3};  // level sensors protecting corresponding heaters
int htrs_en[4] = {0, 0, 0, 0};
int htrs_disable = 0;
int htr_blocked = 0;
unsigned long htrblkd_stime;

void loop_heaters(void) 
{
  float thtr[4];  // for last measured heater temperatures
  thtr[0] = temp0;  thtr[1] = temp0;  thtr[2] = temp1;  thtr[2] = temp1;  // measured temps
  //Serial.println("Entering loop_heaters");

  if( htrs_disable ) {
    for( int i=0; i<4; i++ ) {
      int rhtr = rhtrs[i], lshtr = lshtrs[i];
      RPINS0v[rhtr] = RELAYOFF;  digitalWrite(RPINS0[rhtr], RELAYOFF);
    }
    report_rpins();
    return;
  }
  
  // Safe heaters
  for( int i=0; i<4; i++ ) {
    int rhtr = rhtrs[i], lshtr = lshtrs[i];
    if( digitalRead(RPINS0[rhtr])==RELAYON && digitalRead(LSPINS[lshtr])==LEVELLOW ) {
      RPINS0v[rhtr] = RELAYOFF;  digitalWrite(RPINS0[rhtr], RELAYOFF);  report_rpins();
      btLog("HTR " + String(i) + " OFF because LEV " + String(lshtr) + " low");
      htrblkd_stime = millis();  htr_blocked = 1;  
    }
  }

  // Debounce
  if( htr_blocked ) {
    if( millis()-htrblkd_stime>3000 ) htr_blocked = 0;
    else return;
  }

  // Auto heater functionality
  for( int i=0; i<4; i++ ) {
    int rhtr = rhtrs[i], lshtr = lshtrs[i];
    float temp = thtr[i];
    
    if( htrs_en[rhtr] ) {
      if( temp<temp_setpoint && digitalRead(RPINS0[rhtr])==RELAYOFF && digitalRead(LSPINS[lshtr])==LEVELHIGH ) {
        RPINS0v[rhtr] = RELAYON;  //digitalWrite(RPINS0[rhtr], RELAYON);
        btLog("HTR " + String(i) + " on");
      }
      if( temp>temp_setpoint && digitalRead(RPINS0[rhtr])==RELAYON ) {
        RPINS0v[rhtr] = RELAYOFF;  //digitalWrite(RPINS0[rhtr], RELAYOFF);
        btLog("HTR " + String(i) + " off");
      }
    }
  }
  rpinsv_set();  report_rpins();

  //Serial.println("Leaving loop_heaters");
}


// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

volatile int flow_cnt0, flow_cnt1;
float flow_lpm0, flow_lpm1;
unsigned long flow_lastupdate = 0;
const int flow_measureperiod = 1000;  // millis
void flowISR0(void) { flow_cnt0++; }
void flowISR1(void) { flow_cnt1++; }

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
  float flfreq = (float)flow_cnt0/flowsens_duration*1000.0;  // hertz
  flow_lpm0 = 10.0/82*flfreq;
  flfreq = (float)flow_cnt1/flowsens_duration*1000.0;
  flow_lpm1 = 10.0/82*flfreq;  // 10lpm==82Hz?
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
//   Main auto functionality
// ----------------------------------------------------------------------

enum auto_states {
  STATE_NONE=0, STATE_OFF, STATE_FILL, STATE_WARM, STATE_WASH, STATE_FLUSH, STATE_PAUSE, STATE_SHUT
};
char *auto_statestrs[] = {"NONE", "OFF", "FILL", "WARM", "WASH", "FLUSH", "PAUSE", "SHUT", "WTF"};
auto_states auto_laststate = STATE_NONE;
auto_states auto_state = STATE_OFF;
auto_states auto_nextstate = STATE_OFF;
int auto_substate, auto_lastsubstate;
char *auto_substatestr;
unsigned long auto_statestime, auto_substatestime;

//#include <EEPROM.h>
//const int eepromaddr = 0;
//void auto_switchstate(int state, int substate=0)
//{
//  if( state>=0 ) {
//    auto_state = state;
//    EEPROM.write(eepromaddr, auto_state);
//  }
//  if( substate>=0 ) {
//    auto_substate = substate;
//    EEPROM.write(eepromaddr+1, auto_substate);
//  }
//}
 

void loop_auto(void)
{
  loop_autooff();
  loop_autofill();
  loop_autowarm();
  loop_autowash();
  //loop_autoflush();
  loop_autopause();
  loop_autoshut();
}

// Auto off
enum autooff_states {
  OFF_NONE = 0,
  OFF_RELEASE,
  OFF_DONE
};
const char *autooff_statestrs[] = {"NONE", "RELEASE","DONE","WTF"};
void loop_autooff(void)
{
  if( auto_state!=STATE_OFF ) return;

  // Code to run on transition into main state
  if( auto_laststate!=auto_state ) {
    btLog("Entering state " + String(auto_statestrs[auto_state]));
    auto_nextstate = STATE_FILL;
    auto_substate = auto_lastsubstate = 0;
    auto_substatestr = autooff_statestrs[auto_substate];
    rpinsv_reset();  //pins_off();  
    //for( int i=0; i<8; i++ ) digitalWrite(RPINS1[i], RELAYON);  // release vacuum
    //btLog("Opening valves to release vacuum");
    //delay(500);  pins_off();  // all off
    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
    auto_laststate = auto_state;  
    auto_statestime = millis();
  }

  // Code to run on transition into substate
  if( auto_substate!=auto_lastsubstate ) {
    auto_lastsubstate = auto_substate;
    auto_substatestr = autooff_statestrs[auto_substate];
    auto_substatestime = millis();
    rpinsv_reset();  //pins_off();

    switch( auto_substate ) {
      case OFF_NONE:
        break;

      case OFF_RELEASE:
        RPINS1v[3] = RELAYON;
        break;

      case OFF_DONE:
      break;

    }
    rpinsv_set();  report_rpins();
        
    btLog("Entered substate " + String(auto_substatestr));
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case OFF_NONE:
      auto_substate = OFF_RELEASE;
      break;
      
    case OFF_RELEASE:
      btLog("Checking for release interval elapsed:");
      if( millis()-auto_substatestime>=1000 ) {
        btLog("Setting auto_substate = OFF_DONE");
        auto_substate = OFF_DONE;
      }
      break;

    case OFF_DONE:
      btLog("Entered OFF_DONE");
      break;
  }

  return;
}

// Auto fill
enum autofill_states {
  FILL_NONE = 0,
  FILL_EMPTY,   // run system dry until no scavenge flow
  FILL_FLUSHFROMRES,  // run from reservoir to drain
  FILL_FILLFROMRES  // run from reservoir to work tank
};
const char *autofill_statestrs[] = {"NONE", "EMPTY", "FLUSHFROMRES", "FILLFROMRES", "WTF"};
void loop_autofill(void)
{
  if( auto_state!=STATE_FILL ) return;

  // Code to run on transition into main state
  if( auto_laststate!=auto_state ) {
    btLog("Entering state " + String(auto_statestrs[auto_state]));
    auto_nextstate = STATE_WARM;
    auto_substate = auto_lastsubstate = 0;
    auto_substatestr = autofill_statestrs[auto_substate];
    rpinsv_reset();  //pins_off();  // all off
    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
    auto_laststate = auto_state;  
    auto_statestime = millis();
  }

  // Code to run on transition into substate
  if( auto_substate!=auto_lastsubstate ) {
    auto_lastsubstate = auto_substate;  
    auto_substatestr = autofill_statestrs[auto_substate];
    auto_substatestime = millis();
    rpinsv_reset();  //pins_off();

    switch( auto_substate ) {
      case FILL_NONE:
        break;

      case FILL_EMPTY:  // V1, V4, P1, P2
        //digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[3], RELAYON);
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        RPINS1v[0] = RELAYON;  RPINS1v[3] = RELAYON;
        RPINS0v[4] = RELAYON;  RPINS0v[5] = RELAYON;
        break;

      case FILL_FLUSHFROMRES:  // V2, V4, P1, P2
        //digitalWrite(RPINS1[1], RELAYON);  digitalWrite(RPINS1[3], RELAYON);
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        RPINS1v[1] = RELAYON;  RPINS1v[3] = RELAYON;
        RPINS0v[4] = RELAYON;  RPINS0v[5] = RELAYON;
        break;
        
      case FILL_FILLFROMRES:  // V2, V3, P1, P2
        //digitalWrite(RPINS1[1], RELAYON);  digitalWrite(RPINS1[2], RELAYON);
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        RPINS1v[1] = RELAYON;  RPINS1v[2] = RELAYON;
        RPINS0v[4] = RELAYON;  RPINS0v[5] = RELAYON;
        break;
    }
    rpinsv_set();  report_rpins();

    btLog("Entered substate " + String(auto_substatestr));
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case FILL_NONE:
      auto_substate = FILL_FILLFROMRES;
      break;
      
    case FILL_EMPTY:
      btLog("Checking for no scavenge flow");
      btLog("flow_lasts0==0 and INTERVAL");
      if( flow_lasts0==0 && millis()-flow_lastch0>=8000 ) {
        btLog("8s no scavengef");
        //btLog("FILL_EMPTY: " + millis() + "-" + flow_lastch0);
        
        //auto_substate = FILL_FLUSHFROMRES;
      }
      break;

    case FILL_FLUSHFROMRES:
      btLog("Checking for scavenge flow on fill");
      if( flow_lasts0==1 && millis()-flow_lastch0>=20000 ) {
        btLog("flow_lasts0==1 detected after 20s");
        auto_substate = FILL_FILLFROMRES;
      }
      break;

    case FILL_FILLFROMRES:
      btLog("Checking for working tank full");
      if( digitalRead(LSPINS[0])==0 ) {
        btLog("digitalRead(LSPINS[0])==0 detected");
        auto_substate = FILL_NONE;
        auto_state = auto_nextstate;
      }
      break;
  }

  return;
}

// Auto warm
enum autowarm_states {
  WARM_NONE = 0,
  WARM_WAIT,   // wait with heaters on
  WARM_CYCLE,  // cycle water with heaters on
  WARM_DRAIN,  // drain water with heaters on
};
const char *autowarm_statestrs[] = {"NONE", "WAIT", "CYCLE", "DRAIN", "WTF"};
long int autowarm_cycleinterval = 50000, autowarm_cycleperiod = 6000, autowarm_drainperiod = 4000;
void loop_autowarm(void)
{
  if( auto_state!=STATE_WARM ) return;

  // Code to run on transition into main state
  if( auto_laststate!=auto_state ) {
    btLog("Entering state " + String(auto_statestrs[auto_state]));
    auto_nextstate = STATE_WASH;
    auto_substate = auto_lastsubstate = 0;
    auto_substatestr = autowarm_statestrs[auto_substate];
    rpinsv_reset();  //pins_off();  // all off
    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
    auto_laststate = auto_state;
    auto_statestime = millis();
  }

  // Code to run on transition into substate
  if( auto_substate!=auto_lastsubstate ) {
    auto_lastsubstate = auto_substate;
    auto_substatestr = autowarm_statestrs[auto_substate];
    auto_substatestime = millis();
    rpinsv_reset();  //pins_off();

    switch( auto_substate ) {
      case WARM_NONE:
        break;

      case WARM_WAIT:  // H0, H1
        htrs_en[0] = htrs_en[1] = 1;
        temp_setpoint = temp_reqsetpoint + 4;  // warm to above setpoint
        break;

      case WARM_CYCLE:  // H0, H1, V1, V3, P1, P2
        //digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[2], RELAYON);
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        htrs_en[0] = htrs_en[1] = 1;
        RPINS1v[0] = RELAYON;  RPINS1v[2] = RELAYON;
        RPINS0v[4] = RELAYON;  RPINS0v[5] = RELAYON;
        break;

      case WARM_DRAIN:  // H0, H1, V3, P2
        //digitalWrite(RPINS1[2], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        htrs_en[0] = htrs_en[1] = 1;
        RPINS1v[2] = RELAYON;  RPINS0v[5] = RELAYON;
        break;
    }
    rpinsv_set();  report_rpins();
        
    btLog("Entered substate " + String(auto_substatestr));
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case WARM_NONE:
      auto_substate = WARM_WAIT;
      break;
      
    case WARM_WAIT:
      btLog("Checking for warm interval elapsed:");
      if( millis()-auto_substatestime>=autowarm_cycleinterval ) {
        btLog("Setting auto_substate = WARM_CYCLE");
        auto_substate = WARM_CYCLE;
      }
      break;

    case WARM_CYCLE:
      btLog("Checking for warm cycle complete");
      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_substate = WARM_DRAIN;
      break;
      
    case WARM_DRAIN:
      btLog("Checking for warm drain cycle complete");
      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_substate = WARM_WAIT;
      break;
  }

  return;
}

// Auto wash
enum autowash_states {
  WASH_NONE = 0,
  WASH_CYCLE,
  WASH_TOPUP
};
const char *autowash_statestrs[] = {"NONE", "CYCLE", "TOPUP", "WTF"};  //"PAUSE", 
void loop_autowash(void)
{
  if( auto_state!=STATE_WASH ) return;

  // Code to run on transition into main state
  if( auto_laststate!=auto_state ) {
    btLog("Entering state " + String(auto_statestrs[auto_state]));
    auto_nextstate = STATE_PAUSE;
    auto_substate = auto_lastsubstate = 0;
    auto_substatestr = autowash_statestrs[auto_substate];
    rpinsv_reset();  //pins_off();  // all off
    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
    auto_laststate = auto_state;  
    auto_statestime = millis();
  }

  // Code to run on transition into substate
  if( auto_substate!=auto_lastsubstate ) {
    auto_lastsubstate = auto_substate;
    auto_substatestr = autowash_statestrs[auto_substate];
    auto_substatestime = millis();
    rpinsv_reset();  //pins_off();

    switch( auto_substate ) {
      case WASH_NONE:
        break;

      case WASH_CYCLE:  // H0, H1, V1, V3, P1, P2
        //digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[2], RELAYON);        
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        htrs_en[0] = htrs_en[1] = 1;
        RPINS1v[0] = RPINS1v[2] = RPINS0v[4] = RPINS0v[5] = RELAYON;
        break;

      case WASH_TOPUP:  // H0, H1, V1, V2, V3, P1, P2
        //digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[2], RELAYON);        
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        htrs_en[0] = htrs_en[1] = 1;
        RPINS1v[0] = RPINS1v[1] = RPINS1v[2] = RPINS0v[4] = RPINS0v[5] = RELAYON;
        break;

    }
    rpinsv_set();  report_rpins();
        
    btLog("Entered substate " + String(auto_substatestr));
  }

  // Automatic transitions between substates
  temp_setpoint = temp_reqsetpoint; 
  switch( auto_substate ) {
    case WASH_NONE:
      auto_substate = WASH_CYCLE;
      break;
      
    case WASH_CYCLE:

      // Foot shutoff
      if( millis()-auto_statestime>=5000 ) {
        if( flow_lasts0==0 && millis()-flow_lastch0>5000 || flow_lasts1==0 && millis()-flow_lastch1>5000 ) {
 //       if( flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
          btLog("No flow so leaving WASH_CYCLE");
          auto_state = STATE_PAUSE;
        }
      }

      // Working tank low
      if( digitalRead(LSPINS[1])==1 ) {
        btLog("digitalRead(LSPINS[1])==1 detected");
        auto_substate = WASH_TOPUP;
      }
      break;

    case WASH_TOPUP:
      if( millis()-auto_substatestime>2000 ) {
        btLog("Wash topup timer elapsed");
        auto_substate = WASH_CYCLE;
      }
      break;
    
  }
  
  return;
}   

// Auto pause
enum autopause_states {
  PAUSE_NONE = 0,
  PAUSE_DELAY,
  PAUSE_DRAIN,
  PAUSE_HOLD
};
const char *autopause_statestrs[] = {"NONE", "DELAY", "DRAIN", "HOLD", "WTF"};
void loop_autopause(void)
{
  if( auto_state!=STATE_PAUSE ) return;

  // Code to run on transition into main state
  if( auto_laststate!=auto_state ) {
    btLog("Entering state " + String(auto_statestrs[auto_state]));
    auto_nextstate = STATE_SHUT;
    auto_substate = auto_lastsubstate = 0;
    auto_substatestr = autopause_statestrs[auto_substate];
    rpinsv_reset();  //pins_off();  // all off
    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
    auto_laststate = auto_state;  
    auto_statestime = millis();
  }

  // Code to run on transition into substate
  if( auto_substate!=auto_lastsubstate ) {
    auto_lastsubstate = auto_substate;
    auto_substatestr = autopause_statestrs[auto_substate];
    auto_substatestime = millis();
    rpinsv_reset();  //pins_off();

    switch( auto_substate ) {
      case PAUSE_NONE:
        break;

      case PAUSE_DELAY:  // all off
        break;

      case PAUSE_DRAIN:  // V3, P2
        RPINS1v[2] = RPINS0v[5] = RELAYON;
        break;

      case PAUSE_HOLD:  // all off
        break;
    }
    rpinsv_set();  report_rpins();
        
    btLog("Entered substate " + String(auto_substatestr));
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case PAUSE_NONE:
      auto_substate = PAUSE_DELAY;
      break;

    case PAUSE_DELAY:
      if( millis()-auto_substatestime>5000 ) {
        btLog("Pause delay timer elapsed");
        auto_substate = PAUSE_DRAIN;
      }
      break;

    case PAUSE_DRAIN:
      if( millis()-auto_substatestime>8000 ) {
        btLog("Pause drain timer elapsed");
        auto_substate = PAUSE_HOLD;
      }
      break;
      
    case PAUSE_HOLD:
      break;    
  }
  
  return;
}   

// Auto shut
enum autoshut_states {
  SHUT_NONE = 0,
  SHUT_DRAIN,   // empty working tank to drain
  SHUT_RINSE,  // rinse working tank
};
const char *autoshut_statestrs[] = {"NONE", "DRAIN", "RINSE", "WTF"};
int autoshut_rcycleinterval = 12000, autoshut_rcycleperiod = 5000;
void loop_autoshut(void)
{
  if( auto_state!=STATE_SHUT ) return;

  // Code to run on transition into main state
  if( auto_laststate!=auto_state ) {
    btLog("Entering state " + String(auto_statestrs[auto_state]));
    auto_nextstate = STATE_OFF;
    auto_substate = auto_lastsubstate = 0;
    auto_substatestr = autoshut_statestrs[auto_substate];
    rpinsv_reset();  //pins_off();  // all off
    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
    auto_laststate = auto_state;
    auto_statestime = millis();
  }

  // Code to run on transition into substate
  if( auto_substate!=auto_lastsubstate ) {
    auto_lastsubstate = auto_substate;
    auto_substatestr = autoshut_statestrs[auto_substate];
    auto_substatestime = millis();
    rpinsv_reset();  //pins_off();

    switch( auto_substate ) {
      case SHUT_NONE:
        break;

      case SHUT_DRAIN:  // V1, V4, P1, P2
        //digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[3], RELAYON);
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        RPINS1v[0] = RPINS1v[3] = RPINS0v[4] = RPINS0v[5] = RELAYON;
        break;

      case SHUT_RINSE:  // V1, V3, P1, P2
        //digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[2], RELAYON);
        //digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
        RPINS1v[0] = RPINS1v[2] = RPINS0v[4] = RPINS0v[5] = RELAYON;
        break;
    }
    rpinsv_set();  report_rpins();
        
    btLog("Entered substate " + String(auto_substatestr));
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case SHUT_NONE:
      auto_substate = SHUT_DRAIN;
      break;
      
    case SHUT_DRAIN:
      btLog("Checking for rinse interval elapsed");
      if( millis()-auto_substatestime>=autoshut_rcycleinterval ) {
        if( flow_lasts0==0 && millis()-flow_lastch0>3000 && flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
          btLog("No flow so leaving SHUT");
          auto_state = STATE_OFF;
        }
        else auto_substate = SHUT_RINSE;
      }
      break;

    case SHUT_RINSE:
      btLog("Checking for rinse cycle complete");
      if( millis()-auto_substatestime>=autoshut_rcycleperiod ) auto_substate = SHUT_DRAIN;
      break;
  }

  return;
}


// ----------------------------------------------------------------------
//   Serial command input
// ----------------------------------------------------------------------

// Character-based code
void loop_btserialcmd(void) 
{ 
  char inchar;
  char mess1[32], endchar1 = 'K';
  char mess2[32], endchar2 = 10;

  while( btSerial.available()>0 ) {
    inchar = btSerial.read();
    
    if( inchar==-1 ) { 
      btLog("btSerial.read() returned zero bytes"); 
      break; 
    }

    // Greedy chomp AT message
    if( inchar=='+') {
      Serial.println("Chomping AT message: +");
      int bytesread = btSerial.readBytesUntil(endchar1, mess1, 30);
      mess1[bytesread] = endchar1;  mess1[bytesread+1] = '\0';
      Serial.print(mess1);  
      bytesread = btSerial.readBytesUntil(endchar2, mess2, 30);
      mess2[bytesread] = endchar2;  mess2[bytesread+1] = '\0';
      Serial.print(mess2);  Serial.println("[Chomped]");
      break;
    }

    // Else execute command
    serialcmd(inchar);
  }
}

void serialcmd(char cmd) 
{
  Serial.print("Entered serialcmd with cmd=");  Serial.println(cmd);
  
  // Kill
  if( cmd=='N' ) {
    btLog("KILL");
    setup_pins();
    auto_state = STATE_OFF;
    report_status();
  }

  // Temperature
  if( cmd=='U' || cmd=='V') {
    if( cmd=='U') temp_reqsetpoint += 0.5f;
    else temp_reqsetpoint -= 0.5f;
    report_tsvals();
    btLog("Temp setpoint now " + String(temp_reqsetpoint));
    //Log("Temp setpoint now " + String(temp_reqsetpoint));
  }
  
  // Power relays (manual)
  if( cmd>='f' && cmd<='m' ) {
    int rno = int(cmd) - int('f');  //Serial.println(rno);
    rpinsv_get();
    RPINS0v[rno] = !RPINS0v[rno];  
    //int nval = !digitalRead(RPINS0[rno]);  digitalWrite(RPINS0[rno], nval); 
    btLog("Power relay output " + String(rno) + " to " + String(RPINS0v[rno]));
    rpinsv_set();  report_rpins();
  }

  // Valve relays (manual)
  if( cmd>='F' && cmd<='M' ) {
    int rno = int(cmd) - int('F');  //Serial.println(rno);
    rpinsv_get();
    RPINS1v[rno] = !RPINS1v[rno];  
    //int nval = !digitalRead(RPINS1[rno]);  digitalWrite(RPINS1[rno], nval); 
    btLog("Valve relay output " + String(rno) + " to " + String(RPINS1v[rno]));
    rpinsv_set();  report_rpins();  
  }

  // Advance next state specifier
  if( cmd=='S' ) {
    switch( auto_nextstate ) {
      case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
      case STATE_FILL:  auto_nextstate = STATE_WARM;  break;
      case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
      case STATE_WASH:  auto_nextstate = STATE_PAUSE;  break;
      case STATE_PAUSE:  auto_nextstate = STATE_SHUT;  break;
      case STATE_SHUT:  auto_nextstate = STATE_OFF;  break;
    }
    btLog("Incrementing auto_nextstate to " + String(auto_statestrs[auto_nextstate]));
    report_state();
  }
  
  // Enter next state
  if( cmd=='Y') {
    btLog("Advancing to state " + String(auto_statestrs[auto_nextstate]));
    auto_state = auto_nextstate;
  }

  if( cmd=='P') {
    htrs_disable = !htrs_disable;
    btLog("htrs_disable = " + String(htrs_disable));
  }

  // Advance substate?
  //if( cmd=='Q') {
  //  btLog("Incrementing auto_substate");
  //  auto_substate += 1;  // hope for the best
  //}

  report_status();
}


// ----------------------------------------------------------------------
//   GUI reporting
// ----------------------------------------------------------------------

// Flashing light to indicate bluetooth connection
unsigned long connind_lastupdate = 0;
unsigned long connind_period = 750;
int connind_state = 0;  
void report_connblink() 
{
  String rcmd = "*jR0G0B0*";
  String gcmd = "*jR255G255B0*";

  if( millis()-connind_lastupdate>=connind_period ) {
    connind_lastupdate = millis();
    connind_state = !connind_state;
    if( connind_state ) btSerial.println(rcmd);
    else btSerial.println(gcmd);
  } 
}

void btLog(String mess)
{
  static String lastmess;
  if( mess==lastmess ) return;

  Serial.println("btLog: " + mess);
  btSerial.print("*L" + mess + "\n" + "*");
  lastmess = mess;
}

// Report relay pin outputs
unsigned long rpins_lastreport = 0;
unsigned long rpins_period = 500;
void report_rpins() 
{
  String mstr0 = "*i";
  for( int i=0; i<8; i++ ) {
    if( !digitalRead(RPINS0[i]) ) mstr0 += "1";
    else mstr0 += "0";
  }
  mstr0 += "*";

  String mstr1 = "*I";
  for( int i=0; i<8; i++ ) {
    if( !digitalRead(RPINS1[i]) ) mstr1 += "1";
    else mstr1 += "0";
  }
  mstr1 += "*";

  btSerial.println(mstr0);  //Serial.println(mstr0);
  btSerial.println(mstr1);  //Serial.println(mstr1);

  rpins_lastreport = millis();
}

// Report temperature
unsigned long tsvals_lastreport = 0;
unsigned long tsvals_period = 500;
unsigned long tsready_lastbeep = 0;
unsigned long tsready_beepperiod = 5000;
void report_tsvals() 
{
  String mstr = "*M";
  mstr += String(temp0) + "," + String(temp1) + "*";
  btSerial.println(mstr);

  mstr = "*t";
  mstr += String(temp_reqsetpoint) + "*";
  btSerial.println(mstr);

  if( auto_state==STATE_WARM && temp0>temp_setpoint && millis()-tsready_lastbeep>=tsready_beepperiod ) {
    mstr = "*S*";
    btSerial.println(mstr);
    tsready_lastbeep = millis();
  }

  tsvals_lastreport = millis();
}

// Report analog flow sensor values
unsigned long fsvals_lastreport = 0;
unsigned long fsvals_period = 500;
void report_fsvals() 
{
  String mstr = "*N";
  mstr += String(flow_lpm0) + "," + String(flow_lpm1);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  mstr = "*q";
  if( flow_lasts0==0 ) mstr += 0;
  else mstr += 1;
  if( flow_lasts1==0 ) mstr += 0;
  else mstr += 1;
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  fsvals_lastreport = millis();
}

// Report levels
unsigned long lsvals_lastreport = 0;
unsigned long lsvals_period = 500;
void report_lsvals(void)
{
  String mstr = "*Q";
  for( int i=0; i<4; i++ ) {
    if( digitalRead(LSPINS[i]) ) mstr += "1";
    else mstr += "0";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr0);

  lsvals_lastreport = millis();
}

// Report state
unsigned long state_lastreport = 0;
unsigned long state_period = 200;
void report_state(void)
{
  String mstr = "*s" + String(auto_statestrs[auto_state]) + ": " + auto_substatestr + "*";
  btSerial.println(mstr);
  mstr = "*n" + String(auto_statestrs[auto_nextstate]) + "*";
  btSerial.println(mstr);
  
  state_lastreport = millis();
}

// Report free memory
#include "MemoryFree.h"

unsigned long freemem_lastreport = 0;
unsigned long freemem_period = 20000;
void report_freemem(void)
{
  Serial.print("Free memory: ");  Serial.println(freeMemory());
  
  freemem_lastreport = millis();
}

// Report all
void report_status(void) 
{
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-tsvals_lastreport>=tsvals_period ) report_tsvals();
  if( millis()-fsvals_lastreport>=fsvals_period ) report_fsvals();
  if( millis()-lsvals_lastreport>=lsvals_period ) report_lsvals();
  if( millis()-state_lastreport>=state_period ) report_state();
  if( millis()-freemem_lastreport>=freemem_period ) report_freemem();
}



// ----------------------------------------------------------------------
//   Main setup and loop
// ----------------------------------------------------------------------

void setup() 
{
  btSerial.begin(9600);  Serial.begin(9600);
  btLog("Main setup() called");

  // Hardware detect reset?
  delay(500);
  int rst = digitalRead(RSTSENS);
  btLog("Reset sense pin=" + String(rst));

  // Software detect cause of reset?  Bootloader might reset register
  Serial.print("MCUSR: ");  Serial.println(MCUSR);

  setup_pins();
  setup_tempsens();
  setup_flowsens();
  
  interrupts();

  // Load nvram auto_state and auto_substate???
}

void loop() 
{
  report_connblink();  // indicate connection status

  // Update sensor readings
  loop_tempsens();  // update temperatures
  loop_flowsens();   // update flow sensor values
 
  // Process command input from bluetooth serial
  loop_btserialcmd();

  // Switch heaters (safely) as required
  loop_heaters();

  // Stuff to run for auto
  loop_auto();

  // Timed report
  report_status();

  delay(1);
}

