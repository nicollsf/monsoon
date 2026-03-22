//Fred Nicolls, August 2018


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
//
//      SYSTEM
//
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------

HardwareSerial &btSerial = Serial3;  // pins 14,15
byte nwresets;  // number of warm restarts


// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

const int UNUSEDPINS[16] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52};  // shorted don't drive
const int RPINS0[8] = {23, 25, 27, 29, 31, 33, 35, 37};  // power relay board
const int RPINS1[8] = {39, 41, 43, 45, 47, 49, 51, 53};  // valve relay board
const int LSPINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};  // level sense inputs
const int LSPINSlv[8] = {HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};  // state for low indicator
const int FSPINS[2] = {2, 3};  // flow sensor inputs (on interrupt pins)
const int TSAPIN = A13;  // onewire digital
const int RSTSENS = 6;
const int SP_UND = A9, SP_CLK = A8;  // speed control
const int TRIAC_ZERO = 18, TRIAC_SCR = A10;  // heater control

const int RELAYOFF = HIGH, RELAYON = LOW;
int RPINS0_EN[8], RPINS1_EN[8];  // target values
int rpins_changed;
int rpinsen_reset(void) { for( int i=0; i<8; i++ ) RPINS0_EN[i] = RPINS1_EN[i] = 0; }

void setup_pins()
{
  // Shorted unused pins
  for( int i=0; i<16; i++ ) pinMode(UNUSEDPINS[i], INPUT);

  // Digital outputs for relays (high is off)
  for( int i=0; i<8; i++ ) {
    pinMode(RPINS0[i], OUTPUT);  pinMode(RPINS1[i], OUTPUT);
    digitalWrite(RPINS0[i], RELAYOFF);  digitalWrite(RPINS1[i], RELAYOFF);
    RPINS0_EN[i] = 0;  RPINS1_EN[i] = 0;
  }
  rpins_changed = 0;

  // Digital inputs for flow sensors
  for( int i=0; i<4; i++ ) pinMode(FSPINS[i], INPUT_PULLUP);

  // Level sensors
  for( int i=0; i<8; i++ ) pinMode(LSPINS[i], INPUT_PULLUP);

  // Temperature setup on TSAPIN handled by onewire

  // Heater triac
  pinMode(TRIAC_ZERO, INPUT);  // zero crossing detection (interrupt)
  pinMode(TRIAC_SCR, OUTPUT);  // triac gate control

  // Speed controller
  pinMode(SP_UND, OUTPUT);  // direction
  pinMode(SP_CLK, OUTPUT);  // pulse increment/decrement

  // Reset sense pin
  pinMode(RSTSENS, INPUT);
}


void loop_pins()
{
  rpins_changed = 0;  // will be nonzero on return if any pin changed

  // Pumps
  for ( int i=4; i<8; i++ ) {
    if( digitalRead(RPINS0[i])==RELAYOFF & RPINS0_EN[i]==1 ) {
      digitalWrite(RPINS0[i], RELAYON);
      rpins_changed = 1;
    }
    if( digitalRead(RPINS0[i])==RELAYON & RPINS0_EN[i]==0 ) {
      digitalWrite(RPINS0[i], RELAYOFF);
      rpins_changed = 1;
    }
  }

  // Valves
  for ( int i=0; i<8; i++ ) {
    if( digitalRead(RPINS1[i])==RELAYOFF & RPINS1_EN[i]==1 ) {
      digitalWrite(RPINS1[i], RELAYON);
      rpins_changed = 1;
    }
    if( digitalRead(RPINS1[i])==RELAYON & RPINS1_EN[i]==0 ) {
      digitalWrite(RPINS1[i], RELAYOFF);
      rpins_changed = 1;
    }
  }
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
//   Speed control
// ----------------------------------------------------------------------

// Blocking calls below.  Rather use timer interrupts?

float sc_numpos = 128;  // number of resistor states
float sc_Sp = 100;  // speed output percentage

void speedcontrol_set(float Sp, int force = 0)
{
  // Force low
  if ( force ) {
    digitalWrite(SP_UND, HIGH);
    for ( int i = 0; i < sc_numpos + 10; i++ ) {
      digitalWrite(SP_CLK, HIGH);  delayMicroseconds(3);
      digitalWrite(SP_CLK, LOW);  delayMicroseconds(3);
    }
    btLog("Speed control forced low");
    sc_Sp = 0;
  }

  // Determine changes required
  int cpc = round(sc_Sp / 100 * (sc_numpos - 1));
  int rpc = round(Sp / 100 * (sc_numpos - 1));
  btLog("Speed control:  cpc,rpc=" + String(cpc) + "," + String(rpc));
  if ( rpc == cpc ) return;

  // Modify
  if ( rpc > cpc ) digitalWrite(SP_UND, LOW);
  else digitalWrite(SP_UND, HIGH);
  delayMicroseconds(3);
  btLog("Speed controller pulses: " + String((int)abs((float)rpc - cpc)));
  for ( int i = 0; i < (int)abs((float)rpc - cpc); i++ ) {
    digitalWrite(SP_CLK, HIGH);  delayMicroseconds(3);
    digitalWrite(SP_CLK, LOW);  delayMicroseconds(3);
  }
  sc_Sp = rpc / (sc_numpos - 1) * 100;
}

void setup_speedcontrol(void)
{
  speedcontrol_set(sc_Sp, 1);  // force initial
}

void loop_speedcontrol(void)
{
  //Serial.println("Speed controller: " + String(sc_Sp));  delay(20);
}


// ----------------------------------------------------------------------
//   Heater triac
// ----------------------------------------------------------------------

// Original phase control code moved to attic 3.1.  
// Output pulses as required to track setpoint (assuming gate output reliable)
float triac_reqpulsecount = 0;
float triac_Pp = 0;  // power output percentage

void setup_heatertriac()
{
  // set up zero crossing interrupt
  attachInterrupt(digitalPinToInterrupt(TRIAC_ZERO), zeroCrossingInterrupt, CHANGE);
}

// Interrupt service routines (zero cross TRIAC_ZERO)
void zeroCrossingInterrupt() 
{
  triac_reqpulsecount += triac_Pp/100;  // current pulse deficit

  if( triac_reqpulsecount>=1 ) {
    digitalWrite(TRIAC_SCR, HIGH);
    //Serial.println("TRIAC_SCR high:  " + String(triac_outcount) + "/" + String(triac_reqpulsecount));
    triac_reqpulsecount -= 1.0f;  
  } else {
    digitalWrite(TRIAC_SCR, LOW); 
  }

  if( triac_reqpulsecount>5 ) btLog("WARNING:  large triac pulse deficit triac_reqpulsecount=" + String(triac_reqpulsecount));
}


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int rhtrs[4] = {0, 1, 2, 3};  // heater outputs
int lshtrs[4] = {1, 1, 3, 3};  // level sensors protecting corresponding heaters
int htrs_disable = 0;  // override
int htrs_blocked = 0;
unsigned long htrblkd_stime;
int htrs_changed, htrs_tripped;  // check after call to function


void loop_heaters(void)
{
  htrs_changed = htrs_tripped = 0;
  if ( htrs_blocked && millis() - htrblkd_stime > 3000 ) htrs_blocked = 0; // debounce
  
  // If heaters disabled then force all off and return
  if ( htrs_disable ) {
    for ( int i = 0; i < 4; i++ ) digitalWrite(RPINS0[rhtrs[i]], RELAYOFF);
    return;
  }

  // Safe heaters protected by level sensors
  for ( int i = 0; i < 4; i++ ) {
    int rhtr = rhtrs[i], lshtr = lshtrs[i];
    if ( digitalRead(RPINS0[rhtr])==RELAYON && digitalRead(LSPINS[lshtr])==LSPINSlv[lshtr] ) {
      digitalWrite(RPINS0[rhtr], RELAYOFF);
      btLog("HTR " + String(i) + " OFF because LEV " + String(lshtr) + " low");
      htrs_changed = htrs_tripped = 1;
      htrblkd_stime = millis();  htrs_blocked = 1;  
    }
  }

  // Switch heaters
  for ( int i = 0; i < 4; i++ ) {
    int rhtr = rhtrs[i], lshtr = lshtrs[i];

    //btLog("RPINS0_EN[" + String(rhtr) + "] = " + String(RPINS0_EN[rhtr]));
    if( RPINS0_EN[rhtr]==0 && digitalRead(RPINS0[rhtr])==RELAYON ) {
      digitalWrite(RPINS0[rhtr], RELAYOFF);
      btLog("HTR " + String(i) + " off");
      htrblkd_stime = millis();  htrs_blocked = 1;  htrs_changed = 1;
    }

    if( RPINS0_EN[rhtr] && !htrs_disable && !htrs_blocked ) {
      if( digitalRead(RPINS0[rhtr])==RELAYOFF && digitalRead(LSPINS[lshtr])!=LSPINSlv[lshtr] ) {
        digitalWrite(RPINS0[rhtr], RELAYON);
        btLog("HTR " + String(i) + " on");
        htrs_changed = 1;
      }
    }
  }  
}


// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

volatile int flow_cnt0, flow_cnt1;
float flow_lpm0, flow_lpm1;
unsigned long flow_lastupdate = 0;
const int flow_measureperiod = 1000;  // millis
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
  if ( millis() - flow_lastupdate < flow_measureperiod ) return;

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
  if ( ( flow_lasts0 == 0 && flow_lpm0 >= flow_thr0 ) || ( flow_lasts0 == 1 && flow_lpm0 < flow_thr0 ) ) {
    flow_lasts0 = flow_lpm0 >= flow_thr0;
    flow_lastch0 = flow_lastupdate;
  }
  if ( ( flow_lasts1 == 0 && flow_lpm1 >= flow_thr1 ) || ( flow_lasts1 == 1 && flow_lpm1 < flow_thr1 ) ) {
    flow_lasts1 = flow_lpm1 >= flow_thr1;
    flow_lastch1 = flow_lastupdate;
  }
}


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
//
//      AUTO
//
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
//   Main auto functionality
// ----------------------------------------------------------------------

enum auto_states {
  STATE_NONE = 0, STATE_OFF, STATE_FILL, STATE_WARM, STATE_WASH, STATE_FLUSH, STATE_PAUSE, STATE_SHUT, STATE_CALIB
};
char *auto_statestrs[] = {"NONE", "OFF", "FILL", "WARM", "WASH", "FLUSH", "PAUSE", "SHUT", "CALIB", "WTF"};
auto_states auto_state;
auto_states auto_nextstate = STATE_OFF;
int auto_substate;
char *auto_substatestr;
unsigned long auto_statestime, auto_substatestime;
void auto_switchstate(int state);
void auto_switchsubstate(int substate);
int auto_substatechange = 0;

// Auto topup circuit
int auto_topupenable = 0;
unsigned long auto_topupenstime = 0;
unsigned long auto_topuplasthightime = 0, auto_topuplastlowtime = 0;
void loop_autotopup(void)
{  
  if( !auto_topupenable ) return;
  
  int levellow = digitalRead(LSPINS[0])==LSPINSlv[0];
  if( levellow ) auto_topuplastlowtime = millis();
  else auto_topuplasthightime = millis();

  if( !levellow && millis()-auto_topupenstime>2000 ) {  RPINS0_EN[6] = 0;  return;  }

  if( millis()-auto_topuplasthightime>5000 ) {
    RPINS0_EN[6] = 1;  auto_topupenstime = millis();  // trigger topup  
    btLog("Autotopup triggered");
  }
}

// Auto heater control circuit
int auto_heaterenable = 0;
//int auto_heatermethod = 0;
float temp_setpoint = 45.0;
float temp_reqsetpoint = temp_setpoint;
void loop_autoheater(void)
{
  if( !auto_heaterenable ) return;
   
  // Bang-bang control loop for primary tank
  if( temp0<temp_setpoint ) {
    RPINS0_EN[0] = RPINS0_EN[1] = 1;
  } else {
    RPINS0_EN[0] = RPINS0_EN[1] = 0;
  }
}


void loop_auto(void)
{
  // States
  loop_autooff();
  loop_autofill();
  loop_autowarm();
  loop_autowash();
  loop_autopause();
  loop_autoshut();
  loop_calib();

  // Circuits
  loop_autotopup();
  loop_autoheater();
}

// Auto off
enum autooff_states {
  OFF_NONE = 0,
  OFF_RELEASE,
  OFF_DONE
};
const char *autooff_statestrs[] = {"NONE", "RELEASE", "DONE", "WTF"};
void loop_autooff(void)
{
  if( auto_state!=STATE_OFF ) return;

  // Automatic transitions between substates
  switch( auto_substate ) {
    case OFF_NONE:
      auto_switchsubstate(OFF_DONE);
      break;

    case OFF_RELEASE:
      btLog("Checking for release interval elapsed:");
      if( millis()-auto_substatestime>=1000 ) auto_switchsubstate(OFF_DONE);
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
  FILL_FILLRES,  // fill reservior
  FILL_FLUSHFROMRES,  // run from reservoir to drain
  FILL_FILLFROMRES  // fill working tank from reservoir
};
const char *autofill_statestrs[] = {"NONE", "EMPTY", "FLUSHFROMRES", "FILLFROMRES", "WTF"};
void loop_autofill(void)
{
  if( auto_state!=STATE_FILL ) return;

  // Code to run on transition into substate
  if( auto_substatechange ) {

    switch( auto_substate ) {
      case FILL_EMPTY:  // V1, V4, P1, P2
        RPINS1_EN[0] = 1;  RPINS1_EN[3] = 1;
        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
        break;

      case FILL_FILLRES:  // V6 mains inlet
        RPINS1_EN[5] = 1;
        break;

      case FILL_FLUSHFROMRES:  // V2, V4, P1, P2
        RPINS1_EN[1] = 1;  RPINS1_EN[3] = 1;
        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
        break;

      case FILL_FILLFROMRES:  // V2, V3, P1, P2, P3
        RPINS1_EN[5] = 0;
        RPINS1_EN[1] = 1;  RPINS1_EN[2] = 1;
        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;  RPINS0_EN[6] = 1;
        break;
    }
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case FILL_NONE:
      auto_switchsubstate(FILL_FILLRES);
      break;

    case FILL_EMPTY:
      btLog("Checking for no scavenge flow");
      btLog("flow_lasts0==0 and INTERVAL");
      if( flow_lasts0==0 && millis()-flow_lastch0>=8000 ) {
        btLog("No scavenge flow for 8 seconds");
        //auto_switchstate(0, FILL_FLUSHFROMRES);
      }
      break;

    case FILL_FILLRES:
      btLog("Checking for reservior full");
      if( digitalRead(LSPINS[2])!=LSPINSlv[2] ) {
        btLog("digitalRead(LSPINS[2]) level high detected switching to substate FILL_FILLFROMRES");
        auto_switchsubstate(FILL_FILLFROMRES);
      }
      break;

    case FILL_FLUSHFROMRES:
      btLog("Checking for scavenge flow on fill");
      if( flow_lasts0==1 && millis()-flow_lastch0>=20000 ) {
        btLog("flow_lasts0==1 detected after 20s");
        auto_switchsubstate(FILL_FILLFROMRES);
      }
      break;

    case FILL_FILLFROMRES:
      btLog("Checking for working tank full");
      if( digitalRead(LSPINS[0])!=LSPINSlv[0] ) {
        btLog("digitalRead(LSPINS[0]) level high detected switching to STATE_WARM");
        auto_switchstate(STATE_WARM);
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
long int autowarm_cycleinterval = 50000;
long int autowarm_cycleperiod = 6000;
long int autowarm_drainperiod = 5000;
void loop_autowarm(void)
{
  if( auto_state!=STATE_WARM ) return;
  auto_heaterenable = 1;

  // Code to run on transition into substate
  if( auto_substatechange ) {

    switch( auto_substate ) {
      case WARM_WAIT:
        temp_setpoint = temp_reqsetpoint + 4;  // warm to above setpoint
        break;

      case WARM_CYCLE:  // V1, V3, P1, P2
        RPINS1_EN[0] = 1;  RPINS1_EN[2] = 1;
        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
        break;

      case WARM_DRAIN:  // V3, P2
        RPINS1_EN[2] = 1;  RPINS0_EN[5] = 1;
        break;
    }
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case WARM_NONE:
      auto_switchsubstate(WARM_WAIT);
      break;

    case WARM_WAIT:
      btLog("Checking for warm interval elapsed:");
      if( millis()-auto_substatestime>=autowarm_cycleinterval ) auto_switchsubstate(WARM_CYCLE);
      break;

    case WARM_CYCLE:
      btLog("Checking for warm cycle complete");
      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(WARM_DRAIN);
      break;

    case WARM_DRAIN:
      btLog("Checking for warm drain cycle complete");
      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(WARM_WAIT);
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
  auto_heaterenable = 1;
  auto_topupenable = 1;

  // Code to run on transition into substate
  if( auto_substatechange ) {

    switch( auto_substate ) {
      case WASH_CYCLE:  // H0, H1, V1, V3, P1, P2
        RPINS1_EN[0] = RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = 1;
        break;

      case WASH_TOPUP:  // H0, H1, V1, V2, V3, P1, P2
        RPINS1_EN[0] = RPINS1_EN[1] = RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = RPINS0_EN[6] = 1;
        break;
    }
  }

  // Automatic transitions between substates
  temp_setpoint = temp_reqsetpoint;
  switch( auto_substate ) {
    case WASH_NONE:
      auto_switchsubstate(WASH_CYCLE);
      break;

    case WASH_CYCLE:

      // Foot shutoff
      if( millis()-auto_statestime>=5000 ) {
        //       if( flow_lasts0==0 && millis()-flow_lastch0>5000 || flow_lasts1==0 && millis()-flow_lastch1>5000 ) {
        if( flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
          btLog("No flow so leaving WASH_CYCLE");
          auto_switchstate(STATE_PAUSE);
        }
      }

      // Working tank low
      if( digitalRead(LSPINS[1])==LSPINSlv[1] ) {
        btLog("digitalRead(LSPINS[1]) level low detected");
        auto_switchsubstate(WASH_TOPUP);
      }
      break;

    case WASH_TOPUP:
      if( millis()-auto_substatestime>2000 ) {
        btLog("Wash topup timer elapsed");
        auto_switchsubstate(WASH_CYCLE);
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

  // Code to run on transition into substate
  if( auto_substatechange ) {

    switch( auto_substate ) {
      case PAUSE_DRAIN:  // V3, P2
        RPINS1_EN[2] = RPINS0_EN[5] = 1;
        break;
    }
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case PAUSE_NONE:
      auto_switchsubstate(PAUSE_DELAY);
      break;

    case PAUSE_DELAY:
      if( millis()-auto_substatestime>5000 ) {
        btLog("Pause delay timer elapsed");
        auto_switchsubstate(PAUSE_DRAIN);
      }
      break;

    case PAUSE_DRAIN:
      if( millis()-auto_substatestime>12000 ) {
        btLog("Pause drain timer elapsed");
        auto_switchsubstate(PAUSE_HOLD);
      }
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

  // Code to run on transition into substate
  if( auto_substatechange ) {
    
    switch( auto_substate ) {
      case SHUT_DRAIN:  // V1, V4, P1, P2
        RPINS1_EN[0] = RPINS1_EN[3] = RPINS0_EN[4] = RPINS0_EN[5] = 1;
        break;

      case SHUT_RINSE:  // V1, V3, P1, P2
        RPINS1_EN[0] = RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = 1;
        break;
    }
  }

  // Automatic transitions between substates
  switch( auto_substate ) {
    case SHUT_NONE:
      auto_switchsubstate(SHUT_DRAIN);
      break;

    case SHUT_DRAIN:
      btLog("Checking for rinse interval elapsed");
      if( millis()-auto_substatestime>=autoshut_rcycleinterval ) {
        if( flow_lasts0==0 && millis()-flow_lastch0>3000 && flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
          btLog("No flow so leaving SHUT");
          auto_switchstate(STATE_OFF);
        }
        else auto_switchsubstate(SHUT_RINSE);
      }
      break;

    case SHUT_RINSE:
      btLog("Checking for rinse cycle complete");
      if( millis()-auto_substatestime>=autoshut_rcycleperiod ) auto_switchsubstate(SHUT_DRAIN);
      break;
  }

  return;
}

// ----------------------------------------------------------------------
//   Main calibration functionality
// ----------------------------------------------------------------------

const int hstnsamp = 240;  // heater step test number of samples
const float hstdmsec = 5000;  // heater step test sampling period
float hstsampf[hstnsamp];  // heater step test sample values
unsigned long hststarts;  // heater step test start time
int htlnexti = 0;  // heater step test next sample index
  
// Auto calibration
enum calib_states {
  CALIB_NONE = 0,
  CALIB_SETUPFILL,
  CALIB_SETUPWARM,
  CALIB_SETUPWARMC,
  CALIB_SETUPWARMD,
  CALIB_START,
  CALIB_HEATERONSTEP,
  CALIB_HEATEROFFSTEP,
  CALIB_DONE
};
const char *autocalib_statestrs[] = {"NONE", "SETUPFILL", "SETUPWARM", "SETUPWARMC", "SETUPWARMD", "START", "HEATERONSTEP", "HEATEROFFSTEP", "DONE", "WTF"};
//void loop_calib(void)
//{
//  if( auto_state!=STATE_CALIB ) return;
// 
//  // Code to run on transition into substate
//  if( auto_substatechange ) {
//    triac_Pp = 90;
//    
//    switch( auto_substate ) {
//      case CALIB_SETUPFILL:
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        break;
//
//      case CALIB_SETUPWARM:
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        break;
//
//      case CALIB_SETUPWARMC:
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        RPINS1_EN[2] = 1;  //RPINS1_EN[0] = 1;  
//        break;
//
//      case CALIB_SETUPWARMD:
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        RPINS0_EN[5] = 1;  RPINS1_EN[2] = 1;  
//        break;
//
//      case CALIB_START:
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        break;
//        
//      case CALIB_HEATERONSTEP:
//        triac_Pp = 90;
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        break;
//        
//       case CALIB_HEATEROFFSTEP:
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        break;
//        
//      case CALIB_DONE:
//        btLog("Entered CALIB_DONE");
//        Serial.println("Calibration complete:");
//        Serial.print("hstsampf = [ ");
//        for( int i=0; i<hstnsamp; i++ ) Serial.print(String(hstsampf[i]) + " ");
//        Serial.println("]");
//        break;
//    }
//  }
//
//  // Automatic transitions between substates
//  switch( auto_substate ) {
//    case CALIB_NONE:
//      auto_switchsubstate(CALIB_SETUPFILL);
//      break;
//
//    case CALIB_SETUPFILL:
//      if( millis()-auto_substatestime>=10000 && millis()-auto_topuplastlowtime>=10000 ) {
//        btLog("Switching from CALIB_SETUPFILL to CALIB_SETUPWARM");
//        auto_switchsubstate(CALIB_SETUPWARM);
//      }
//      break;
//
//    case CALIB_SETUPWARM:
//      if( temp0>temp_setpoint-5 ) {
//        btLog("Switching from CALIB_SETUPWARM to CALIB_START because " + String(temp0) + ">" + String(temp_setpoint-5));
//        auto_switchsubstate(CALIB_START);
//      }
//      if( auto_substate==CALIB_SETUPWARM && millis()-auto_substatestime>=autowarm_cycleinterval ) auto_switchsubstate(CALIB_SETUPWARMC);
//      break;
//
//    case CALIB_SETUPWARMC:
//      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(CALIB_SETUPWARMD);
//      break;
//
//    case CALIB_SETUPWARMD:
//      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(CALIB_SETUPWARM);
//      break;
//
//    case CALIB_START:
//      if( millis()-auto_substatestime>=30000 && millis()-auto_topuplastlowtime>=30000 ) {  // stabilise
//        btLog("Switching from CALIB_SETUP to CALIB_HEATERONSTEP");
//        hststarts = millis();  htlnexti = 0;  // start capture
//        auto_switchsubstate(CALIB_HEATERONSTEP);
//      }
//      break;
//
//    case CALIB_HEATERONSTEP:  
//      btLog("In CALIB_HEATERONSTEP(temp=" + String(temp0) + "): htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
//      if( htlnexti>=hstnsamp ) {  auto_switchsubstate(CALIB_DONE);  return; }
//      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
//      if( temp0>temp_setpoint+5 ) auto_switchsubstate(CALIB_HEATEROFFSTEP);
//      break;
//
//    case CALIB_HEATEROFFSTEP:  
//      btLog("In CALIB_HEATEROFFSTEP:  htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
//      if( htlnexti>=hstnsamp ) { auto_switchsubstate(CALIB_DONE);  return; }
//      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
//      if( temp0<temp_setpoint-5 ) auto_switchsubstate(CALIB_HEATERONSTEP);
//      break;
//  }
//
//  return;
//}


void loop_calib(void)
{
  if( auto_state!=STATE_CALIB ) return;
 
  switch( auto_substate ) {
    
    case CALIB_NONE:
      auto_switchsubstate(CALIB_SETUPFILL);
      break;

    case CALIB_SETUPFILL:
      RPINS1_EN[2] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_topupenable = 1;
        
      if( millis()-auto_substatestime>=10000 && millis()-auto_topuplastlowtime>=10000 ) {
        btLog("Switching from CALIB_SETUPFILL to CALIB_SETUPWARM");
        auto_switchsubstate(CALIB_SETUPWARM);
      }
      break;

    case CALIB_SETUPWARM:
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
   
      if( temp0>temp_setpoint-5 ) {
        btLog("Switching from CALIB_SETUPWARM to CALIB_START because " + String(temp0) + ">" + String(temp_setpoint-5));
        auto_switchsubstate(CALIB_START);
      }
      if( auto_substate==CALIB_SETUPWARM && millis()-auto_substatestime>=autowarm_cycleinterval ) auto_switchsubstate(CALIB_SETUPWARMC);
      break;

    case CALIB_SETUPWARMC:
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      RPINS1_EN[2] = 1;  //RPINS1_EN[0] = 1;  
      
      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(CALIB_SETUPWARMD);
      break;

    case CALIB_SETUPWARMD:
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
      RPINS0_EN[5] = 1;  RPINS1_EN[2] = 1;  
      
      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(CALIB_SETUPWARM);
      break;

    case CALIB_START:
      RPINS1_EN[2] = 1;  
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_topupenable = 1;
      
      if( millis()-auto_substatestime>=30000 && millis()-auto_topuplastlowtime>=30000 ) {  // stabilise
        btLog("Switching from CALIB_SETUP to CALIB_HEATERONSTEP");
        hststarts = millis();  htlnexti = 0;  // start capture
        auto_switchsubstate(CALIB_HEATERONSTEP);
      }
      break;

    case CALIB_HEATERONSTEP:  
      triac_Pp = 90;
      RPINS1_EN[2] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_topupenable = 1;
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
              
      btLog("In CALIB_HEATERONSTEP(temp=" + String(temp0) + "): htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
      if( htlnexti>=hstnsamp ) {  auto_switchsubstate(CALIB_DONE);  return; }
      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
      if( temp0>temp_setpoint+5 ) auto_switchsubstate(CALIB_HEATEROFFSTEP);
      break;

    case CALIB_HEATEROFFSTEP:  
      RPINS1_EN[2] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_topupenable = 1;
          
      btLog("In CALIB_HEATEROFFSTEP:  htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
      if( htlnexti>=hstnsamp ) { auto_switchsubstate(CALIB_DONE);  return; }
      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
      if( temp0<temp_setpoint-5 ) auto_switchsubstate(CALIB_HEATERONSTEP);
      break;

    case CALIB_DONE:
      if( auto_substatechange ) {
        btLog("Calibration completed:");
        Serial.print("hstsampf = [ ");
        for( int i=0; i<hstnsamp; i++ ) Serial.print(String(hstsampf[i]) + " ");
        Serial.println("]");
      }
      break;

      
  }

  return;
}


#include <EEPROM.h>  // persistence
const int eepromaddr0 = 0;  // base offset
void auto_switchsubstate(int substate)
{
  auto_substate = substate;
  auto_topupenable = auto_heaterenable = 0;
  switch( auto_state ) {
    case STATE_OFF:  auto_substatestr = autooff_statestrs[auto_substate];  break;
    case STATE_FILL:  auto_substatestr = autofill_statestrs[auto_substate];  break;
    case STATE_WARM:  auto_substatestr = autowarm_statestrs[auto_substate];  break;
    case STATE_WASH:  auto_substatestr = autowash_statestrs[auto_substate];  break;
    case STATE_PAUSE:  auto_substatestr = autopause_statestrs[auto_substate];  break;
    case STATE_SHUT:  auto_substatestr = autoshut_statestrs[auto_substate];  break;
    case STATE_CALIB:  auto_substatestr = autocalib_statestrs[auto_substate];  break;
  }
  btLog("Entering substate " + String(auto_substatestr));
  auto_substatestime = millis();
  rpinsen_reset();

  // Call auto with substatechange flag
  auto_substatechange = 1;
  loop_auto();
  auto_substatechange = 0;
}

void auto_switchstate(int state)
{  
  // Update total wash time when leaving wash
  if ( auto_state==STATE_WASH && state!=STATE_WASH ) {
    unsigned long auto_totalwashtime;
    EEPROM.get(eepromaddr0 + 2, auto_totalwashtime);
    auto_totalwashtime += millis() - auto_statestime;
    EEPROM.put(eepromaddr0 + 2, auto_totalwashtime);
  }

  // New state persistent store
  auto_state = state;
  auto_statestime = millis();
  btLog("Entering state " + String(auto_statestrs[auto_state]));
  byte bauto_state = (byte)auto_state;
  EEPROM.put(eepromaddr0, bauto_state);
  auto_substate = 0;

  switch( auto_state ) {
    case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
    case STATE_FILL:  auto_nextstate = STATE_WARM;  break;
    case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
    case STATE_WASH:  auto_nextstate = STATE_PAUSE;  break;
    case STATE_PAUSE:  auto_nextstate = STATE_SHUT;  break;
    case STATE_SHUT:  auto_nextstate = STATE_OFF;  break;
    case STATE_CALIB:  auto_nextstate = STATE_OFF;  break;
  }
}



// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
//
//      GUI
//
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------


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
      mess1[bytesread] = endchar1;  mess1[bytesread + 1] = '\0';
      Serial.print(mess1);
      bytesread = btSerial.readBytesUntil(endchar2, mess2, 30);
      mess2[bytesread] = endchar2;  mess2[bytesread + 1] = '\0';
      Serial.print(mess2);  Serial.println("[Chomped]");
      break;
    }

    // Else execute command
    serialcmd(inchar);
  }
}

// Commands
// N kill; f-m power relays 1-8; F-M valve relays 1-8; U-V temp setpoint up/down
// S next next state; Y advance next state; P pause heaters
// A/a heater U/D; B/b speed U/D
void serialcmd(char cmd)
{
  Serial.print("Entered serialcmd with cmd=");  Serial.println(cmd);

  // Kill
  if ( cmd == 'N' ) {
    btLog("KILL");
    setup_pins();
    auto_switchstate(STATE_OFF);  //auto_state = STATE_OFF;
    report_status();
  }

  // Temperature
  if ( cmd == 'U' || cmd == 'V') {
    if ( cmd == 'U') temp_reqsetpoint += 1.0f;
    else temp_reqsetpoint -= 1.0f;
    report_tsvals();
    btLog("Temp setpoint now " + String(temp_reqsetpoint));
    //Log("Temp setpoint now " + String(temp_reqsetpoint));
  }

  // Power relays (manual)
  if ( cmd >= 'f' && cmd <= 'm' ) {
    int rno = int(cmd) - int('f');  //Serial.println(rno);
    //rpinsv_get();
    btLog("Before:  RPINS0_EN[" + String(rno) + "] = " + String(RPINS0_EN[rno]));
    RPINS0_EN[rno] = !RPINS0_EN[rno];
    btLog("After:  RPINS0_EN[" + String(rno) + "] = " + String(RPINS0_EN[rno]));
    //int nval = !digitalRead(RPINS0[rno]);  digitalWrite(RPINS0[rno], nval);
    //btLog("Power relay output " + String(rno) + " to " + String(RPINS0_EN[rno]));
    //rpinsv_set();  report_rpins();
    
  }

  // Valve relays (manual)
  if ( cmd >= 'F' && cmd <= 'M' ) {
    int rno = int(cmd) - int('F');  //Serial.println(rno);
    //rpinsv_get();
    RPINS1_EN[rno] = !RPINS1_EN[rno];
    //int nval = !digitalRead(RPINS1[rno]);  digitalWrite(RPINS1[rno], nval);
    btLog("Output RPINS1_EN[" + String(rno) + "] to " + String(RPINS1_EN[rno]));
    //rpinsv_set();  report_rpins();
  }

  // Heater power
  if ( cmd == 'A' || cmd == 'a') {
    float nPpi;
    if ( cmd == 'A') {
      nPpi = triac_Pp + 5;  if ( nPpi>100 ) nPpi = 100;
    } else {
      nPpi = triac_Pp - 5;  if ( nPpi<0 ) nPpi = 0;
    }
    triac_Pp = nPpi;
    btLog("Heater power output " + String(triac_Pp));
  }

  // Speed control
//  if ( 0 && cmd == 'B' || cmd == 'b') {
//    float nSp = sc_Sp;
//    btLog("Speed control currently " + String(nSp));
//    if ( cmd == 'B') nSp += 5;
//    else nSp -= 5;
//    if ( nSp < 0 ) nSp = 0;
//    if ( nSp > 100 ) nSp = 100;
//    speedcontrol_set(nSp,1);
//    btLog("Speed control set to " + String(nSp));
//  }

  if ( cmd == 'B' || cmd == 'b') {
    if( cmd=='B') {
      btLog("Speed control 10 up");
      digitalWrite(SP_UND, HIGH);
    }
    else {
      btLog("Speed control 10 down");
      digitalWrite(SP_UND, LOW);
    }
    for ( int i = 0; i < 10; i++ ) {
      digitalWrite(SP_CLK, HIGH);  delayMicroseconds(3);
      digitalWrite(SP_CLK, LOW);  delayMicroseconds(3);
    }
  }

  // Advance next state specifier
  if ( cmd == 'S' ) {
    switch ( auto_nextstate ) {
      case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
      case STATE_FILL:  auto_nextstate = STATE_WARM;  break;
      case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
      case STATE_WASH:  auto_nextstate = STATE_PAUSE;  break;
      case STATE_PAUSE:  auto_nextstate = STATE_SHUT;  break;
      case STATE_SHUT:  auto_nextstate = STATE_CALIB;  break;
      case STATE_CALIB:  auto_nextstate = STATE_OFF;  break;
    }
    btLog("Incrementing auto_nextstate to " + String(auto_statestrs[auto_nextstate]));
    report_state();
  }

  // Enter next state
  if ( cmd == 'Y') {
    btLog("Advancing to state " + String(auto_statestrs[auto_nextstate]));
    auto_switchstate(auto_nextstate);  //auto_state = auto_nextstate;
  }

  // Toggle heater disable
  if ( cmd == 'P') {
    htrs_disable = !htrs_disable;
    btLog("htrs_disable = " + String(htrs_disable));
  }

  report_status();
}


// ----------------------------------------------------------------------
//   GUI reporting
// ----------------------------------------------------------------------

// j alive light; i power relay; I valve relay; q flow sense; Q level sense; L log window
// w restarts; M temp graphs; N flow graphs
// t temp setpoint; S beep; n next state; s current state and substate

// Flashing light to indicate bluetooth connection
unsigned long connind_lastupdate = 0;
unsigned long connind_period = 750;
int connind_state = 0;
void report_connblink()
{
  String rcmd = "*jR0G0B0*";
  String gcmd = "*jR255G255B0*";

  if ( millis() - connind_lastupdate >= connind_period ) {
    connind_lastupdate = millis();
    connind_state = !connind_state;
    if ( connind_state ) btSerial.println(rcmd);
    else btSerial.println(gcmd);
  }
}

void btLog(String mess)
{
  static String lastmess;
  if ( mess == lastmess ) return;

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
  for ( int i = 0; i < 8; i++ ) {
    if ( !digitalRead(RPINS0[i]) ) mstr0 += "1";
    else mstr0 += "0";
  }
  mstr0 += "*";

  String mstr1 = "*I";
  for ( int i = 0; i < 8; i++ ) {
    if ( !digitalRead(RPINS1[i]) ) mstr1 += "1";
    else mstr1 += "0";
  }
  mstr1 += "*";

  if( 0 ) {
    String mstr2 = "";
    for ( int i = 0; i < 8; i++ ) {
      if ( RPINS0_EN[i]==1 ) mstr2 += "1";
      else mstr2 += "0";
    }
    Serial.println("RPINS0_EN = " + mstr2);
  }

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
  if ( flow_lasts0 == 0 ) mstr += 0;
  else mstr += 1;
  if ( flow_lasts1 == 0 ) mstr += 0;
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
  for ( int i = 0; i < 4; i++ ) {
    if ( digitalRead(LSPINS[i]) == LSPINSlv[i] ) mstr += "0";
    else mstr += "1";
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

  //String mstr = "*a";
  //for( int i=0; i<4; i++ ) {
  //  if( digitalRead(LSPINS[i]) ) mstr += "1";
  //  else mstr += "0";
  //}
  //mstr += "*";
  //btSerial.println(mstr);  //Serial.println(mstr0);

  state_lastreport = millis();
}

// Report free memory
#include "MemoryFree.h"

unsigned long progstat_lastreport = 0;
unsigned long progstat_period = 20000;
void report_progstat(void)
{
  byte bauto_state = EEPROM.read(eepromaddr0);
  byte nwresets = EEPROM.read(eepromaddr0 + 1);
  unsigned long twashtime;
  EEPROM.get(eepromaddr0 + 2, twashtime);

  Serial.println("Program status:");
  Serial.print("  Free memory: ");  Serial.println(freeMemory());
  Serial.print("  Current state: ");  Serial.println(auto_statestrs[auto_state]);
  Serial.print("  Current saved state: ");  Serial.println(auto_statestrs[(auto_states)bauto_state]);
  Serial.print("  Warm reset count: ");  Serial.println((int)nwresets);
  Serial.print("  Total wash time (s): ");  Serial.println(twashtime / 1000.0f);
  if( auto_state==STATE_CALIB ) {
    Serial.print("  Total calibration time (s): ");  Serial.println((millis()-auto_substatestime) / 1000.0f);
  }
  Serial.println("Total wash time (s): " + String(twashtime / 1000.0f));

  String mstr = "*w" + String((int)nwresets) + "*";
  btSerial.println(mstr);  // warm reset count

  progstat_lastreport = millis();
}

// Debug status
unsigned long debugstat_lastreport = 0;
unsigned long debugstat_period = 30000;
void report_debugstat(void)
{
  //Serial.println("temp0,temp1=" + String(temp0) + "," + String(temp1) + " (temp_setpoint=" + String(temp_setpoint) + ")");

  Serial.println("Calibration state:");
  Serial.print("hstsampf = [ ");
  for( int i=0; i<htlnexti; i++ ) Serial.print(String(hstsampf[i]) + " ");
  Serial.println("]");

  debugstat_lastreport = millis();
}


// Report all
void report_status(void)
{
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-tsvals_lastreport>=tsvals_period ) report_tsvals();
  if( millis()-fsvals_lastreport>=fsvals_period ) report_fsvals();
  if( millis()-lsvals_lastreport>=lsvals_period ) report_lsvals();
  if( millis()-state_lastreport>=state_period ) report_state();
  if( millis()-progstat_lastreport>=progstat_period ) report_progstat();
  if( millis()-debugstat_lastreport>=debugstat_period ) report_debugstat();
}


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
//
//      MAIN
//
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
//   Main setup and loop
// ----------------------------------------------------------------------

#include <avr/wdt.h>

void setup()
{
  wdt_disable();

  int serialm = 0;
  switch( serialm ) {
    case 0:
      btSerial.begin(115200);
      Serial.begin(115200);
      break;
    case 1:
      btSerial.begin(9600);
      Serial.begin(9600);
      break;
  }
  btLog("Main setup() called");
  
  // Hardware detect reset
  int rst = digitalRead(RSTSENS);
  btLog("Reset sense pin=" + String(rst));

  setup_pins();
  setup_tempsens();
  setup_flowsens();
  setup_heatertriac();
  setup_speedcontrol();

  interrupts();

  // Enable once and run to reset nonvolatiles
  if ( 0 ) {
    EEPROM.put(eepromaddr0, (byte)0);
    EEPROM.put(eepromaddr0 + 1, (byte)0);
    EEPROM.put(eepromaddr0 + 2, (unsigned long)0);
  }

  // Initial state
  auto_state = STATE_NONE;
  if ( rst == LOW ) {
    auto_switchstate(STATE_OFF);
    EEPROM.put(eepromaddr0 + 1, (byte)0);
  }
  else {  // warm reset detected
    byte bauto_state = EEPROM.read(eepromaddr0);
    auto_switchstate((auto_states)bauto_state);
    nwresets = EEPROM.read(eepromaddr0 + 1) + 1;
    EEPROM.put(eepromaddr0 + 1, nwresets); // count
    btLog("Warm reset number " + String(nwresets));
    btLog("Forced return to state " + String(auto_statestrs[(auto_states)bauto_state]));
  }

  triac_Pp = 80;  // include in saved state?  and control loop?

  wdt_enable(WDTO_2S);     // enable watchdog
}

void loop()
{
  wdt_reset();  // still alive
  report_connblink();  // indicate connection status

  // Update sensor readings
  loop_tempsens();  // update temperatures
  loop_flowsens();   // update flow sensor values

  // Process command input from bluetooth serial
  loop_btserialcmd();

  // Stuff to run for auto
  loop_auto();

  // Actuation
  loop_heaters();
  loop_speedcontrol();
  loop_pins();
  if( rpins_changed || htrs_changed ) rpins_lastreport = 0;  // force report

  // Timed report
  report_status();

  delay(10);
}
