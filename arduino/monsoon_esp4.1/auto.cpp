#include "auto.h"
#include "calib.h"
#include "system.h"
#include "sensors.h"
#include "control.h"
#include "gui.h"
#include <Preferences.h>


// ----------------------------------------------------------------------
//   Main auto functionality
// ----------------------------------------------------------------------

int auto_double = 1;  // main auto system mode
const char *auto_statestrs[] = {"NONE", "OFF", "FILL", "SETUP", "SETUP1", "WARM", "WARM1", "WASH", "WASH1", "FLUSHE", "FLUSHR", "PAUSE", "SHUT", "EMPTY", "CALIBP", "CALIBT", "CALIBDUMP", "WTF"};
auto_states auto_state = STATE_NONE;
auto_states auto_nextstate = STATE_OFF;
const char *autonone_statestrs[] = {"NONE", "WTF"};
const char **auto_substatestrs = autonone_statestrs;
int auto_substate = 0;
//char *auto_substatestr;
unsigned long auto_statestime, auto_substatestime;
//int auto_substatechange = 0;



// ----------------------------------------------------------------------
//   Auto circuits
// ----------------------------------------------------------------------

int auto_woverflowstopenable = 0;
unsigned long auto_woverflowstopstime = 0;
bool auto_woverflowstop_active = false;
int auto_woverflowstop_prev_en = ROFF;

void loop_autowoverflowstop(void)
{
  if( !auto_woverflowstopenable ) return;
  if( millis()-auto_woverflowstopstime<500 ) return;  // limit frequency

  // Temporarily block recovery pump when tank high
  if( level_high1 ) {
    if( !auto_woverflowstop_active && getpump_en(RPUMPR)==RON ) {
      auto_woverflowstop_prev_en = getpump_en(RPUMPR);
      setpump_en(RPUMPR, ROFF);
      auto_woverflowstop_active = true;
      auto_woverflowstopstime = millis();
    }
  } else {
    // Level no longer high so restore previous
    if( auto_woverflowstop_active ) {
      setpump_en(RPUMPR, auto_woverflowstop_prev_en);
      auto_woverflowstop_active = false;
      auto_woverflowstopstime = millis();
    }
  }
}

int auto_woverflowopenenable = 0;
unsigned long auto_woverflowopenstime = 0;

void loop_autowoverflowopen(void)
{
  if( !auto_woverflowopenenable ) return;

  if( getrelay_en(RPDRAIN)==RON && millis()-auto_woverflowopenstime>2000 ) setrelay_en(RPDRAIN, ROFF);  // end open
  if( millis()-auto_woverflowopenstime<4000 ) return;  // limit frequency
  
  if( level_high1 ) {
    if( getrelay_en(RPDRAIN)==ROFF ) btLog("In loop_autowoverflow: overflow so opening drain");
    setrelay_en(RPDRAIN, RON);
    auto_woverflowopenstime = millis();
  }
}


// Auto topup working tank
int auto_wtopupenable = 0;
unsigned long auto_wtopupopenstime = 0;
//unsigned long auto_wtopuplasthightime = 0, auto_wtopuplastlowtime = 0;

void loop_autowtopup(void)
{
  if( !auto_wtopupenable ) return;

  if( getrelay_en(0)==RON && (level_high1 || millis()-auto_wtopupopenstime>1500) ) setrelay_en(0, ROFF);  // end open
  if( millis()-auto_wtopupopenstime<10000 ) return;  // limit frequency

  // Open inlet if level low
  if( !level_high1 ) {
    setrelay_en(0, RON);
    auto_wtopupopenstime = millis();
  }
}


// Auto burp working tank
int auto_wburpenable = 0;
unsigned long auto_wburpopenstime = 0;
//unsigned long auto_wburplasthightime = 0, auto_wburplastlowtime = 0;

void loop_autowburp(void)
{
  if( auto_double || !auto_wburpenable ) return;

  if( digitalRead(RPINS[6])!=RPINS_ROFF[6] && millis()-auto_wburpopenstime>500 ) RPINS_EN[6] = RPINS_ROFF[6];  // end burp
  if( millis()-auto_wburpopenstime<4000 ) return;  // limit frequency

  // Start burp if level high
  if( !level_high1 ) {
    digitalWrite(RPINS[6],!RPINS_ROFF[6]);
    auto_wburpopenstime = millis();
  }
}


// Auto burp if over pressure
int auto_wbleedenable = 0;
unsigned long auto_wbleedopenstime = 0;

hw_timer_t *Timer0_Cfg = NULL;

void IRAM_ATTR Timer0_ISR(void)
{
  if( auto_double || !auto_wbleedenable ) return;

  if( getrelay_en(RPBURP)!=RPINS_ROFF[RPBURP] ) {
    digitalWrite(RPINS[RPBURP], RPINS_ROFF[RPBURP]);  // do this urgently!
    setrelay_en(RPBURP, RPINS_ROFF[RPBURP]);  // end bleed

    timerStop(Timer0_Cfg);
  }
}

void setup_autowbleed()
{
  //pinMode(LED, OUTPUT);
  Serial.println("Entering setup_autowbleed");
  Timer0_Cfg = timerBegin(10000);
  if( Timer0_Cfg==NULL ) {
    Serial.println("setup_autowbleed: timerBegin failed");  //delay(1000);
  }
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 1000, true, 0);
  timerStop(Timer0_Cfg);
}

void loop_autowbleed(void)
{
  const int timerintflag = 1;  // Use timer interrupt to end bleed

  if( auto_double || !auto_wbleedenable || getrelay_en(6)!=RPINS_ROFF[6] ) return;

  // End bleed if necessary
  if( !timerintflag ) {
    if( getrelay_en(RPBURP)!=RPINS_ROFF[RPBURP] && millis()-auto_wbleedopenstime>50 ) {
      digitalWrite(RPINS[RPBURP], RPINS_ROFF[RPBURP]);  // do this urgently!
      setrelay_en(RPBURP, RPINS_ROFF[RPBURP]);  // end bleed
    }
  }

  // Start bleed
  if( psens0>175.0 && millis()-auto_wbleedopenstime>100 ) {
    Serial.println("loop_autowbleed: over pressure (" + String(psens0) + " kPa)");
    auto_wbleedopenstime = millis();
    digitalWrite(RPINS[RPBURP], !RPINS_ROFF[RPBURP]);  // do this urgently!
    setrelay_en(RPBURP, !RPINS_ROFF[RPBURP]);

    if( timerintflag ) {
      Serial.println("loop_autowbleed: calling timerStart");
      timerWrite(Timer0_Cfg, 0);
      timerStart(Timer0_Cfg);
    }
  }

}


// Auto overflow working tank
int auto_scavengecontrolenable = 0;
unsigned long auto_scavengecontrolstime = 0;

void loop_autoscavengecontrol(void)
{
  if( !auto_scavengecontrolenable ) return;

  if( level_high1 && flow_lpm0>flow_lpm1 && millis()-auto_scavengecontrolstime>=1000 ) {
    setpump_perc(0, sc_setperc[0]-4);
    auto_scavengecontrolstime = millis();
    return;
  }
    
  if( millis()-auto_scavengecontrolstime>=1000 ) {
    if( flow_lpm0<flow_lpm1 ) setpump_perc(0, sc_setperc[0]-1);
    else setpump_perc(0, sc_setperc[0]+1);
    auto_scavengecontrolstime = millis();
  }
}


// ----------------------------------------------------------------------
//   Auto states
// ----------------------------------------------------------------------

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
  auto_substatestrs = autooff_statestrs;

  // Handle substates
  switch( auto_substate ) {
    case OFF_NONE:
      auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = auto_scavengecontrolenable = 0;
      auto_wbleedenable = 1;  // safe pressure when under manual control
      setup_pins();
      auto_switchsubstate(OFF_RELEASE);
      break;

    case OFF_RELEASE:
      setrelay_en(RPBURP, RON);
      if( millis()-auto_substatestime>5000 && psens0<15.0 ) auto_switchsubstate(OFF_DONE);
      break;

    case OFF_DONE:
      break;
  }

  return;
}

// Auto fill
enum autofill_states {
  FILL_NONE = 0,
  FILL_EMPTY,  // run system dry until no scavenge flow
  FILL_HALF,  // half fill working tank from mains
  FILL_FULL,  // full fill working tank from mains
  FILL_DONE
};
const char *autofill_statestrs[] = {"NONE", "EMPTY", "HALF", "FULL", "DONE", "WTF"};
void loop_autofill(void)
{
  if( auto_state!=STATE_FILL ) return;
  auto_substatestrs = autofill_statestrs;

  // Handle substates
  switch( auto_substate ) {
    case FILL_NONE:
      btLog("Entering FILL_NONE");
      auto_wbleedenable = 0;
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = 0;  // disable auto circuits
      auto_switchsubstate(FILL_FULL);
      break;

    case FILL_EMPTY:
      btLog("loop_autofill:  FILL_EMPTY not implemented");
      break;

    case FILL_HALF:  // half fill working tank from main inlet
      setrelay_en(RPINLET, RON);

      if( level_high0 ) auto_switchsubstate(FILL_FULL);
      break;

    case FILL_FULL:  // full fill working tank from main inlet
      auto_wbleedenable = 1;
      setrelay_en(RPINLET, RON);

      if( level_high1 ) auto_switchsubstate(FILL_DONE);
      break;

    case FILL_DONE:   
      if( auto_double ) {
        btLog("FILL_DONE so switching to STATE_SETUP");
        auto_switchstate(STATE_SETUP);
      } else {
        btLog("FILL_DONE so switching to STATE_SETUP1");
        auto_switchstate(STATE_SETUP1);
      }
      break;
  }

  return;
}

// Auto setup
enum autosetup_states {
  SETUP_NONE = 0,
  SETUP_INIT,
  SETUP_STABILISE,
  SETUP_TOPUP,
  SETUP_DONE
};
const char *autosetup_statestrs[] = {"NONE", "INIT", "STABILISE", "TOPUP", "DONE", "WTF"};
void loop_autosetup(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_SETUP ) return;
  auto_substatestrs = autosetup_statestrs;

  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Handle substates
  switch( auto_substate ) {
    case SETUP_NONE:
      btLog("Entering SETUP_NONE");
      auto_switchsubstate(SETUP_INIT);
      break;

    case SETUP_INIT:
      if( just_entered ) {
        btLog("Entering SETUP_INIT");
        setpump_perc(RPUMPR, 100);  setpump_perc(RPUMPD, 40);
        setrelay_en(RPDELIVER, RON);  setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON);  
        auto_wtopupenable = 1;
      }
      //Serial.println("flow_lastlt0,flow_lastlt1=" + String(flow_lastlt0) + "," + String(flow_lastlt1) + " (millis=" + String(millis()) + ")");
      if( millis()-flow_lastlt1>20000 ) {
        btLog("Scavenge flow detected so advancing to STABILISE");
        auto_switchsubstate(SETUP_STABILISE);
      }
      break;

    case SETUP_STABILISE:
      if( just_entered ) {
        btLog("Entering SETUP_STABILISE");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON); 
        auto_wtopupenable = 1;
      }
      if( level_high1 ) auto_switchsubstate(SETUP_DONE);
      break;

    case SETUP_TOPUP:
      btLog("Entering SETUP_TOPUP");
      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON); 
      setrelay_en(RPINLET, RON);
      if( level_high1 ) auto_switchsubstate(SETUP_DONE);
      break;

    case SETUP_DONE:   
      btLog("SETUP_DONE so switching to STATE_WARM");
      auto_switchstate(STATE_WARM);
      break;
  }

  return;
}

// Auto warm
enum autowarm_states {
  WARM_NONE = 0,
  WARM_CYCLE,  // cycle water with heaters on
  WARM_WAIT,   // wait with heaters on
  WARM_DRAIN,  // drain water with heaters on
};
const char *autowarm_statestrs[] = {"NONE", "CYCLE", "WAIT", "DRAIN", "WTF"};
long int autowarm_waitperiod = 30000;
long int autowarm_cycleperiod = 10000;
long int autowarm_drainperiod = 10000;
void loop_autowarm(void)
{
  if( auto_state!=STATE_WARM ) return;
  auto_substatestrs = autowarm_statestrs;
  
  //temp_setpoint = temp_reqsetpoint + 4;  // warm to above setpoint

  // Handle substates
  switch( auto_substate ) {
    case WARM_NONE:
      btLog("Entering WARM_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      temp_controlmode = TCHEATER;  // bang-bang temperature control
      auto_switchsubstate(WARM_CYCLE);
      break;

    case WARM_WAIT:      
      //if( auto_double ) setpump_en(RPUMPD, RON);
      //setpump_en(RPUMPR, RON);
      //setrelay_en(RPDELIVER, RON);
      //auto_heaterenable = 1;  
      //setrelay_en(RPHEATER, RON);  setrelay_en(RPHEATERA, RON);  
      
      btLog("Checking for warm interval elapsed");
      if( millis()-auto_substatestime>=autowarm_waitperiod ) auto_switchsubstate(WARM_CYCLE);
      break;

    case WARM_CYCLE:
      if( auto_double ) setpump_en(RPUMPD, RON);
      setpump_en(RPUMPR, RON);
      setrelay_en(RPDELIVER, RON);
      //auto_heaterenable = 1;  
      //setrelay_en(RPHEATER, RON);  setrelay_en(RPHEATERA, RON);  

      btLog("Checking for warm cycle complete");
      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(WARM_DRAIN);
      break;

    case WARM_DRAIN:     
      setpump_en(RPUMPR, RON);
      //setrelay_en(RPDELIVER, RON);
      //auto_heaterenable = 1;  
      //setrelay_en(RPHEATER, RON);  setrelay_en(RPHEATERA, RON);  
        
      btLog("Checking for warm drain cycle complete");
      if( millis()-auto_substatestime>=autowarm_drainperiod || level_high1 ) auto_switchsubstate(WARM_WAIT);
      break;
  }

  return;
}

// Auto wash
enum autowash_states {
  WASH_NONE = 0,
  WASH_CYCLE
};
const char *autowash_statestrs[] = {"NONE", "CYCLE", "WTF"};
void loop_autowash(void)
{
  if( auto_state!=STATE_WASH ) return;
  auto_substatestrs = autowash_statestrs;

  // Handle substates
  //temp_setpoint = temp_reqsetpoint;
  switch( auto_substate ) {
    case WASH_NONE:
      btLog("Entering WASH_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      auto_switchsubstate(WASH_CYCLE);
      break;

    case WASH_CYCLE:
      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);  
      if( auto_double ) setpump_en(RPUMPD, RON);
      //auto_heaterenable = 1;  
      //setrelay_en(RPHEATER, RON);  setrelay_en(RPHEATERA, RON);  
      //auto_scavengecontrolenable = 1;
      
      // // Foot advance
      // if( millis()-auto_statestime>=6000 ) {
      //   if( flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
      //     btLog("No flow so advancing to STATE_calib");
      //     auto_switchstate(STATE_FLUSHE);
      //   }
      // }
      break;
  }

  return;
}

// Auto shut
enum autoshut_states {
  SHUT_NONE = 0,
  SHUT_DRAINTOSAFE,  // empty working tank to drain
  SHUT_DRAINTOEMPTY,  // empty working tank to drain
  SHUT_RINSE,  // rinse working tank
  SHUT_DONE
};
const char *autoshut_statestrs[] = {"NONE", "DRAINTOSAFE", "DRAINTOEMPTY", "RINSE", "DONE", "WTF"};
int autoshut_rcycleinterval = 12000, autoshut_rcycleperiod = 5000;
void loop_autoshut(void)
{
  if( auto_state!=STATE_SHUT ) return;
  auto_substatestrs = autoshut_statestrs;
  
  // Automatic transitions between substates
  switch( auto_substate ) {
    case SHUT_NONE:
      btLog("Entering SHUT_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      auto_switchsubstate(SHUT_DRAINTOSAFE);
      break;

    case SHUT_DRAINTOSAFE:
      setrelay_en(RPDRAIN, RON);  
      setpump_en(RPUMPR, RON);
      if( auto_double ) setpump_en(RPUMPD, RON);
      
      if( !level_high0 ) {
        btLog("Level low so leaving DRAINTOSAFE");
        auto_switchsubstate(SHUT_DONE);
      }

      break;

    case SHUT_DONE:
      auto_switchstate(STATE_OFF);
      break;
  }

  return;
}

// Auto empty
enum autoempty_states {
  EMPTY_NONE = 0,
  EMPTY_DRAIN,
  EMPTY_DONE
};
const char *autoempty_statestrs[] = {"NONE", "DRAIN", "DONE", "WTF"};
void loop_autoempty(void)
{
  if( auto_state!=STATE_EMPTY ) return;
  auto_substatestrs = autoempty_statestrs;
  
  // Automatic transitions between substates
  switch( auto_substate ) {
    case EMPTY_NONE:
      btLog("Entering EMPTY_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      auto_switchsubstate(EMPTY_DRAIN);
      break;

    case EMPTY_DRAIN:
      setrelay_en(RPDRAIN, RON);  
      setpump_en(RPUMPR, RON);  
      if( auto_double ) setpump_en(RPUMPD, RON); 

      if( millis()-auto_substatestime>20000 && psens0<15.0 && millis()-flow_lastht0>5000 && millis()-flow_lastht1>5000 ) {
        btLog("EMPTY_DRAIN detected finish");
        auto_switchsubstate(EMPTY_DONE);
      }
      break;

    case EMPTY_DONE:
      auto_switchstate(STATE_OFF);
      break;
  }

  return;
}


// ----------------------------------------------------------------------
//   Auto states (single pump)
// ----------------------------------------------------------------------

// Auto setup
enum autosetup1_states {
  SETUP1_NONE = 0,
  SETUP1_INIT,
  SETUP1_TOPUP,
  SETUP1_INIT2,
  SETUP1_TOPUP2,
  SETUP1_RUN,
  SETUP1_DONE
};
const char *autosetup1_statestrs[] = {"NONE", "INIT", "TOPUP", "INIT2", "TOPUP2", "RUN", "DONE", "WTF"};
void loop_autosetup1(void)
{
  if( auto_state!=STATE_SETUP1 ) return;
  auto_substatestrs = autosetup1_statestrs;

  // Handle substates
  switch( auto_substate ) {
    case SETUP1_NONE:
      btLog("Entering SETUP1_NONE");
      auto_wbleedenable = 1;  // bleed overpressure
      auto_switchsubstate(SETUP1_INIT);
      break;

    case SETUP1_INIT:
      btLog("Entering SETUP1_INIT");
      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);  
      //Serial.println("flow_lastlt0,flow_lastlt1=" + String(flow_lastlt0) + "," + String(flow_lastlt1) + " (millis=" + String(millis()) + ")");
      if( millis()-flow_lastlt1>5000 ) {
        btLog("Scavenge flow detected so advancing to TOPUP");
        auto_switchsubstate(SETUP1_TOPUP);
      }
      break;

    case SETUP1_TOPUP:
      btLog("Entering SETUP1_TOPUP");
      setrelay_en(RPINLET, RON);
      if( level_high1 ) auto_switchsubstate(SETUP1_INIT2);
      break;

     case SETUP1_INIT2:
      btLog("Entering SETUP1_INIT2");
      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);  
      //Serial.println("flow_lastlt0,flow_lastlt1=" + String(flow_lastlt0) + "," + String(flow_lastlt1) + " (millis=" + String(millis()) + ")");
      if( millis()-flow_lastlt1>5000 ) {
        btLog("Scavenge flow detected so advancing to TOPUP2");
        auto_switchsubstate(SETUP1_TOPUP2);
      }
      break;

    case SETUP1_TOPUP2:
      btLog("Entering SETUP1_TOPUP2");
      setrelay_en(RPINLET, RON);
      if( level_high1 ) auto_switchsubstate(SETUP1_RUN);
      break;     
      
    case SETUP1_RUN:
      btLog("Entering SETUP_RUN");
      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON); 
      if( millis()-auto_substatestime>120000 ) auto_switchsubstate(SETUP1_DONE);
      break;

    case SETUP1_DONE:   
      btLog("SETUP1_DONE so switching to STATE_WARM1");
      auto_switchstate(STATE_WARM1);
      break;
  }

  return;
}

// Auto warm1 (single pump)
enum autowarm1_states {
  WARM1_NONE = 0,
  WARM1_CYCLE,  // cycle water with heaters on
  WARM1_WAIT,   // wait with heaters on
  WARM1_DRAIN,  // drain water with heaters on
};
const char *autowarm1_statestrs[] = {"NONE", "CYCLE", "WAIT", "DRAIN", "WTF"};
long int autowarm1_waitperiod = 30000;
long int autowarm1_cycleperiod = 10000;
long int autowarm1_drainperiod = 15000;
void loop_autowarm1(void)
{
  if( auto_state!=STATE_WARM1 ) return;
  auto_substatestrs = autowarm1_statestrs;
  
  //temp_setpoint = temp_reqsetpoint + 4;  // warm to above setpoint

  // Handle substates
  switch( auto_substate ) {
    case WARM1_NONE:
      btLog("Entering WARM1_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      temp_controlmode = TCHEATER;
      auto_switchsubstate(WARM1_WAIT);
      break;

    case WARM1_WAIT:      
      if( auto_double ) setpump_en(RPUMPR, RON);
      
      btLog("Checking for warm interval elapsed");
      if( millis()-auto_substatestime>=autowarm1_waitperiod ) auto_switchsubstate(WARM_CYCLE);
      break;

    case WARM1_CYCLE:
      if( auto_double ) setpump_en(RPUMPR, RON);

      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);
      if( auto_double ) setpump_en(RPUMPD, RON);

      btLog("Checking for warm cycle complete");
      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(WARM1_DRAIN);
      break;

    case WARM1_DRAIN:     
      setpump_en(RPUMPR, RON);
        
      btLog("Checking for warm drain cycle complete");
      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(WARM1_WAIT);
      break;
  }

  return;
}

// Auto wash1 (single pump)
enum autowash1_states {
  WASH1_NONE = 0,
  WASH1_CYCLE
};
const char *autowash1_statestrs[] = {"NONE", "CYCLE", "WTF"};
void loop_autowash1(void)
{
  if( auto_state!=STATE_WASH1 ) return;
  auto_substatestrs = autowash1_statestrs;

  // Handle substates
  //temp_setpoint = temp_reqsetpoint;
  switch( auto_substate ) {
    case WASH1_NONE:
      btLog("Entering WASH1_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      temp_controlmode = TCHEATER;
      auto_switchsubstate(WASH1_CYCLE);
      break;

    case WASH1_CYCLE:
      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);  
      if( auto_double ) setpump_en(RPUMPD, RON);
      
      // // Foot advance
      // if( millis()-auto_statestime>=6000 ) {
      //   if( flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
      //     btLog("No flow so advancing to STATE_FLUSHE");
      //     auto_switchstate(STATE_FLUSHE);
      //   }
      // }
      break;
  }

  return;
}


// ----------------------------------------------------------------------
//   Main auto loop
// ----------------------------------------------------------------------

void setup_auto(void)
{
  Serial.println("setup_auto: calling setup_autowbleed");
  setup_autowbleed();
}


void loop_auto(void)
{
  if( auto_state==STATE_NONE) auto_switchstate(STATE_OFF);

  // States
  loop_autooff();
  loop_autofill();  
  loop_autosetup();
  loop_autowarm();
  loop_autowarm1();
  loop_autowash();
  loop_autowash1();
  // loop_autoflushempty();
  // loop_autoflushrefill();
  // loop_autopause();
  loop_autoshut();
  loop_autoempty();
  loop_calib();

  // Circuits
  loop_autowoverflowstop();
  loop_autowoverflowopen();
  loop_autowtopup();
  loop_autowburp();
  loop_autowbleed();
  loop_autoscavengecontrol();
  //loop_autoheater();
}


// ----------------------------------------------------------------------
//   State transition methods
// ----------------------------------------------------------------------

#include <EEPROM.h>  // persistence
const int eepromaddr0 = 0;  // base offset
void auto_switchsubstate(int substate)
{
  auto_substate = substate;
  const char *auto_substatestr = auto_substatestrs[auto_substate];  // handler must set auto_substatestrs

  btLog("Entering substate " + String(auto_substatestr));
  auto_substatestime = millis();
  rpinsen_reset();
  pumpsen_reset();
  //auto_heaterenable = 0;
  if( auto_double ) setrelay_en(RPBURP, RON);

  // Call auto with substatechange flag
  loop_auto();
}

void auto_switchstate(int state)
{  
  // Update total wash time when leaving wash
  if( auto_state==STATE_WASH && state!=STATE_WASH ) {
    unsigned long auto_totalwashtime;
    EEPROM.get(eepromaddr0 + 2, auto_totalwashtime);
    auto_totalwashtime += millis() - auto_statestime;
    EEPROM.put(eepromaddr0 + 2, auto_totalwashtime);
  }

  // New state persistent store
  auto_state = (auto_states)state;
  auto_statestime = millis();
  btLog("Entering state " + String(auto_statestrs[auto_state]));
  byte bauto_state = (byte)auto_state;
  EEPROM.put(eepromaddr0, bauto_state);
  auto_substate = 0;

  rpinsen_reset();
  switch( auto_state ) {
    case STATE_NONE:  auto_nextstate = STATE_OFF;  break;
    case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
    case STATE_FILL:  auto_nextstate = STATE_SETUP;  break;
  //    if( auto_double ) auto_nextstate = STATE_SETUP;  
  //    else auto_nextstate = STATE_SETUP1;
  //    break;
    case STATE_SETUP:  auto_nextstate = STATE_WARM;  break;
    case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
    case STATE_WASH:  auto_nextstate = STATE_PAUSE;  break;
    case STATE_PAUSE:  auto_nextstate = STATE_SHUT;  break;
    case STATE_SHUT:  auto_nextstate = STATE_EMPTY;  break;
    case STATE_EMPTY:  auto_nextstate = STATE_CALIBP;  break;
    case STATE_CALIBP:  auto_nextstate = STATE_CALIBT;  break;
    case STATE_CALIBT:
      auto_nextstate = STATE_CALIBDUMP;
      // Enforce default intent for CALIBT exactly once on transition
      setpump_perc(RPUMPR, 100);
      setpump_en(RPUMPR, RON);
      break;
    case STATE_CALIBDUMP:  auto_nextstate = STATE_OFF;  break;

    case STATE_SETUP1:  auto_nextstate = STATE_WARM1;  break;
    case STATE_WARM1:  auto_nextstate = STATE_WASH1;  break;
    case STATE_WASH1:  auto_nextstate = STATE_PAUSE;  break;
  }

  // Disable auto circuits
  auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;
}

void auto_advancestate(void) 
{ 
  auto_switchstate(auto_nextstate);
}