#include "auto.h"
#include "system.h"
#include "sensors.h"
#include "control.h"
#include "gui.h"


// ----------------------------------------------------------------------
//   Main auto functionality
// ----------------------------------------------------------------------

char *auto_statestrs[] = {"NONE", "OFF", "FILL", "WARM", "WASH", "FLUSHE", "FLUSHR", "PAUSE", "SHUT", "CALIB", "WTF"};
auto_states auto_state;
auto_states auto_nextstate = STATE_OFF;
int auto_substate;
char *auto_substatestr;
char **auto_substatestrs;
unsigned long auto_statestime, auto_substatestime;
int auto_substatechange = 0;


// ----------------------------------------------------------------------
//   Auto topup circuits
// ----------------------------------------------------------------------

// Auto topup working tank
int auto_wtopupenable = 0;
unsigned long auto_wtopupenstime = 0;
unsigned long auto_wtopuplasthightime = 0, auto_wtopuplastlowtime = 0;

void loop_autowtopup(void)
{  
  if( !auto_wtopupenable ) return;

  int wlevellow = digitalRead(LSPINS[0])==LSPINSlv[0];
  if( wlevellow ) auto_wtopuplastlowtime = millis();
  else auto_wtopuplasthightime = millis();

  if( !wlevellow && millis()-auto_wtopupenstime>1000 ) RPINS0_EN[6] = 0;
  if( wlevellow && millis()-auto_wtopuplasthightime>5000 ) {
    if( RPINS0_EN[6]==0 ) btLog("Autotopup working tank triggered");
    RPINS0_EN[6] = 1;  auto_wtopupenstime = millis();  // trigger topup  
  }
}

// Auto topup reservoir
int auto_rtopupenable = 0;
unsigned long auto_rtopupenstime = 0;
unsigned long auto_rtopuplasthightime = 0, auto_rtopuplastlowtime = 0;

void loop_autortopup(void)
{  
  if( !auto_rtopupenable ) return;

  int rlevellow = digitalRead(LSPINS[2])==LSPINSlv[2];
  if( rlevellow ) auto_rtopuplastlowtime = millis();
  else auto_rtopuplasthightime = millis();

  if( !rlevellow && millis()-auto_rtopupenstime>1000 ) RPINS1_EN[5] = 0;
  if( rlevellow && millis()-auto_rtopuplasthightime>5000 ) {
    if( RPINS1_EN[5]==0 ) btLog("Autotopup reservoir triggered");
    RPINS1_EN[5] = 1;  auto_rtopupenstime = millis();  // trigger topup  
  }
}


// ----------------------------------------------------------------------
//   Auto heater circuits
// ----------------------------------------------------------------------

float temp_setpoint = 45.0;
float temp_reqsetpoint = temp_setpoint;

// Auto heater working tank
int auto_wheaterenable = 0;
//int auto_wheatermethod = 0;
void loop_autowheater(void)
{
  if( !auto_wheaterenable ) return;
  triac_Pp = 85;  // fix for now
   
  // Bang-bang control loop for working tank
  if( temp0<temp_setpoint ) {
    RPINS0_EN[0] = RPINS0_EN[1] = 1;
  } else {
    RPINS0_EN[0] = RPINS0_EN[1] = 0;
  }
}

// Auto heater reservoir 
int auto_rheaterenable = 0;
//int auto_rheatermethod = 0;
void loop_autorheater(void)
{
  if( !auto_rheaterenable ) return;
   
  // Bang-bang control loop for reservoir
  if( temp1<temp_setpoint ) {
    //if( RPINS0_EN[0]==0 | RPINS0_EN[1]==0 ) RPINS0_EN[2] = 1;  // limit power
  } else {
    //RPINS0_EN[2] = 0;
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
      auto_rtopupenable = auto_wtopupenable = auto_rheaterenable = auto_wheaterenable = 0;
      setup_pins();
      auto_switchsubstate(OFF_RELEASE);
      break;

    case OFF_RELEASE:
      RPINS1_EN[2] = 1;
      btLog("Checking for release interval elapsed:");
      if( millis()-auto_substatestime>=1500 ) {
        auto_switchsubstate(OFF_DONE);
      }
      break;

    case OFF_DONE:
      RPINS1_EN[2] = 0;
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
  FILL_FILLWORK,  // fill working tank and reservoir
  FILL_POSTFILLRES,  // fill reservior final
  FILL_FLUSHFROMRES,  // run from reservoir to drain
  FILL_FILLFROMRES  // fill working tank from reservoir
};
const char *autofill_statestrs[] = {"NONE", "EMPTY", "FILLRES", "FILLWORK", "POSTFILLRES", "FLUSHFROMRES", "FILLFROMRES", "WTF"};
void loop_autofill(void)
{
  if( auto_state!=STATE_FILL ) return;
  auto_substatestrs = autofill_statestrs;

  // Handle substates
  switch( auto_substate ) {
    case FILL_NONE:
      auto_rtopupenable = auto_wtopupenable = auto_rheaterenable = auto_wheaterenable = 0;
      auto_switchsubstate(FILL_FILLRES);
      break;

    case FILL_FILLRES:
      auto_rtopupenable = 1;
      
      //btLog("Checking for reservior full");
      if( digitalRead(LSPINS[2])!=LSPINSlv[2] ) {
        btLog("Reservoir full so switching to substate FILL_FILLWORK");
        auto_switchsubstate(FILL_FILLWORK);
      }
      break;

    case FILL_FILLWORK:  // fill working tank while running
      auto_wtopupenable = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;  RPINS1_EN[2] = 1;  

      if( digitalRead(LSPINS[0])!=LSPINSlv[0] ) auto_switchsubstate(FILL_POSTFILLRES);
      break;

    case FILL_POSTFILLRES:        
      if( digitalRead(LSPINS[2])!=LSPINSlv[2] ) {
        btLog("Reservoir full so switching to STATE_WARM");
        auto_rtopupenable = 0;
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
long int autowarm_cycleinterval = 30000;
long int autowarm_cycleperiod = 10000;
long int autowarm_drainperiod = 5000;
void loop_autowarm(void)
{
  if( auto_state!=STATE_WARM ) return;
  auto_substatestrs = autowarm_statestrs;
  
  temp_setpoint = temp_reqsetpoint + 4;  // warm to above setpoint

  // Handle substates
  switch( auto_substate ) {
    case WARM_NONE:
      auto_rtopupenable = auto_wtopupenable = auto_rheaterenable = auto_wheaterenable = 1;
      auto_switchsubstate(WARM_WAIT);
      break;

    case WARM_WAIT:
      btLog("Checking for warm interval elapsed");
      if( millis()-auto_substatestime>=autowarm_cycleinterval ) auto_switchsubstate(WARM_CYCLE);
      break;

    case WARM_CYCLE:
      RPINS1_EN[0] = RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  // V1, V3, P1, P2
      
      btLog("Checking for warm cycle complete");
      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(WARM_DRAIN);
      break;

    case WARM_DRAIN:
      RPINS1_EN[2] = RPINS0_EN[5] = 1;  // V3, P2
      
      btLog("Checking for warm drain cycle complete");
      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(WARM_WAIT);
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
  temp_setpoint = temp_reqsetpoint;
  switch( auto_substate ) {
    case WASH_NONE:
      auto_rtopupenable = 0;  auto_wtopupenable = 1;  auto_rheaterenable = auto_wheaterenable = 1;
      auto_switchsubstate(WASH_CYCLE);
      break;

    case WASH_CYCLE:
      RPINS1_EN[0] = RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  //  V1, V3, P1, P2
      
      // Foot advance
      if( millis()-auto_statestime>=6000 ) {
        if( flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
          btLog("No flow so advancing to STATE_FLUSHE");
          auto_switchstate(STATE_FLUSHE);
        }
      }
      break;
  }

  return;
}


// Auto flushempty
enum autoflushempty_states {
  FLUSHEMPTY_NONE = 0,
  FLUSHEMPTY_SIGNAL,
  FLUSHEMPTY_CYCLE,
  FLUSHEMPTY_DELAY,
  FLUSHEMPTY_DONE
};
const char *autoflushempty_statestrs[] = {"NONE", "SIGNAL", "CYCLE", "DELAY", "WTF"};
void loop_autoflushempty(void)
{
  if( auto_state!=STATE_FLUSHE ) return;
  auto_substatestrs = autoflushempty_statestrs;

  // Handle substates
  switch( auto_substate ) {
    case FLUSHEMPTY_NONE:
      auto_rtopupenable = auto_wtopupenable = 0;  auto_rheaterenable = auto_wheaterenable = 1;
      //RPINS1_EN[0] = RPINS1_EN[2] = 0;
      RPINS1_EN[3] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  //  V4, P1, P2
      auto_switchsubstate(FLUSHEMPTY_SIGNAL);
      break;

    case FLUSHEMPTY_SIGNAL:
      RPINS0_EN[4] = 0;  RPINS1_EN[3] = RPINS0_EN[5] = 1;  // P1, P2

      // Transition to main state
      if( millis()-auto_substatestime>=1000 ) {
        auto_switchsubstate(FLUSHEMPTY_CYCLE);
      }
      
      break;
      
    case FLUSHEMPTY_CYCLE:
      //RPINS1_EN[0] = RPINS1_EN[2] = 0;
      RPINS1_EN[3] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  // V4, P1, P2
      if( digitalRead(LSPINS[1])==LSPINSlv[1] ) auto_switchsubstate(FLUSHEMPTY_DELAY);
            
      // Foot advance
      if( millis()-auto_statestime>=4000 ) {
        if( flow_lasts1==0 && millis()-flow_lastch1>3000 ) {
          btLog("No flow so advancing to STATE_PAUSE");
          auto_switchstate(STATE_PAUSE);
        }
      }
      break;

    case FLUSHEMPTY_DELAY:
      RPINS1_EN[3] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  // V4, P1, P2
      if( millis()-auto_substatestime>=5000 ) {
        auto_switchstate(STATE_FLUSHR);
      }
      break;
    
  }

  return;
}


// Auto flushrefill
enum autoflushrefill_states {
  FLUSHREFILL_NONE = 0,
  FLUSHREFILL_CYCLE,
  FLUSHREFILL_DONE
};
const char *autoflushrefill_statestrs[] = {"NONE", "CYCLE", "WTF"};
void loop_autoflushrefill(void)
{
  if( auto_state!=STATE_FLUSHR ) return;
  auto_substatestrs = autoflushrefill_statestrs;

  // Handle substates
  switch( auto_substate ) {
    case FLUSHREFILL_NONE:
      auto_rtopupenable = auto_wtopupenable = 0;  auto_rheaterenable = auto_wheaterenable = 1;
      auto_switchsubstate(FLUSHREFILL_CYCLE);
      break;
    
    case FLUSHREFILL_CYCLE:
      RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  RPINS1_EN[3] = 0;  
      auto_wtopupenable = 1; 
      if( digitalRead(LSPINS[0])!=LSPINSlv[0] ) auto_switchstate(STATE_WASH);
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
  auto_substatestrs = autopause_statestrs;
  
  // Automatic transitions between substates
  switch( auto_substate ) {
    case PAUSE_NONE:
      auto_rtopupenable = auto_wtopupenable = auto_wheaterenable = auto_rheaterenable = 0;
      auto_switchsubstate(PAUSE_DELAY);
      break;

    case PAUSE_DELAY:
      if( millis()-auto_substatestime>8000 ) {
        btLog("Pause delay timer elapsed");
        auto_switchsubstate(PAUSE_DRAIN);
      }
      break;

    case PAUSE_DRAIN:
      RPINS1_EN[2] = RPINS0_EN[5] = 1;  // V3, P2
      
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
  auto_substatestrs = autoshut_statestrs;
  
  // Automatic transitions between substates
  switch( auto_substate ) {
    case SHUT_NONE:
      auto_rtopupenable = auto_wtopupenable = auto_wheaterenable = auto_rheaterenable = 0;
      auto_switchsubstate(SHUT_DRAIN);
      break;

    case SHUT_DRAIN:
      RPINS1_EN[0] = RPINS1_EN[3] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  // V1, V4, P1, P2
      
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
      RPINS1_EN[0] = RPINS1_EN[2] = RPINS0_EN[4] = RPINS0_EN[5] = 1;  // V1, V3, P1, P2
      
      btLog("Checking for rinse cycle complete");
      if( millis()-auto_substatestime>=autoshut_rcycleperiod ) auto_switchsubstate(SHUT_DRAIN);
      break;
  }

  return;
}


// ----------------------------------------------------------------------
//   Main auto loop
// ----------------------------------------------------------------------

void loop_auto(void)
{
  // States
  loop_autooff();
  loop_autofill();
  loop_autowarm();
  loop_autowash();
  loop_autoflushempty();
  loop_autoflushrefill();
  loop_autopause();
  loop_autoshut();
//  loop_calib();

  // Circuits
  loop_autowtopup();
  loop_autortopup();
  loop_autowheater();
  loop_autorheater();
}


// ----------------------------------------------------------------------
//   State transition methods
// ----------------------------------------------------------------------

#include <EEPROM.h>  // persistence
const int eepromaddr0 = 0;  // base offset
void auto_switchsubstate(int substate)
{
  auto_substate = substate;
  auto_substatestr = auto_substatestrs[auto_substate];  // handler must set auto_substatestrs

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
