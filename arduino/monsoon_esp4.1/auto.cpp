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
const char *auto_statestrs[] = {"NONE", "OFF", "FILL", "SETUP1", "WARM", "WARM1", "WASH", "WASH1", "RINSE", "FLUSHE", "FLUSHR", "PAUSE", "SHUT", "CALIBP", "CALIBT", "CALDUMP", "WTF"};
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
  if( tank_full ) {
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
  
  if( tank_full ) {
    if( getrelay_en(RPDRAIN)==ROFF ) btLog("In loop_autowoverflow: overflow so opening drain");
    setrelay_en(RPDRAIN, RON);
    auto_woverflowopenstime = millis();
  }
}


// Auto topup working tank
int auto_wtopupenable = 0;
unsigned long auto_wtopup_interval = 10000;
unsigned long auto_wtopupopenstime = 0;
unsigned long auto_wtopuplasthightime = 0;
int auto_wtopup_count = 0;
bool wash_temp_stabilised = false;
unsigned long wash_temp_stable_start_time = 0;

void loop_autowtopup(void)
{
  if( !auto_wtopupenable ) {
    auto_wtopuplasthightime = millis(); // Keep timer reset while disabled
    return;
  }

  if( tank_full ) {
    auto_wtopuplasthightime = millis();
  }

  // If the relay was turned on manually (e.g. via GUI) but the timer wasn't 
  // updated, snap the timer to the current time so the manual override 
  // gets the full 2-second grace period instead of instantly turning off.
  if( getrelay_en(RPINLET)==RON && (millis() - auto_wtopupopenstime > 10000) ) {
      auto_wtopupopenstime = millis();
  }

  // If the inlet relay is on, check if it needs to be turned off.
  if( getrelay_en(RPINLET)==RON ) {
    // Turn off if tank is full, or if a 2s auto-pulse has finished
    if( tank_full || (millis() - auto_wtopupopenstime > 2000) ) {
      setrelay_en(RPINLET, ROFF);  // end open
    }
  }

  if( millis()-auto_wtopupopenstime<auto_wtopup_interval ) return;  // limit frequency

  // Open inlet if it's been too long since tank was high AND we are trying to recover
  if( tank_empty && (millis() - auto_wtopuplasthightime > auto_wtopup_interval) ) {
    if( getpump_en(RPUMPR) == RON && getrelay_en(RPINLET) == ROFF ) {
      if( auto_state == STATE_WASH ) {
        auto_wtopup_count++;
        btLog("Auto-topup triggered (" + String(auto_wtopup_count) + ")");
      } else {
        btLog("Auto-topup: Pan empty/not filling, injecting water burst.");
      }
      setrelay_en(RPINLET, RON);
      auto_wtopupopenstime = millis();
    } else {
      static unsigned long last_dbg = 0;
      if(millis() - last_dbg > 5000) {
        last_dbg = millis();
        Serial.printf("Auto-topup DBG: Blocked. RPUMPR_en=%d, RPINLET_en=%d\n", 
                      getpump_en(RPUMPR), getrelay_en(RPINLET));
      }
    }
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
  if( tank_empty ) {
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


// Auto scavenge control
int auto_scavengecontrolenable = 0;
unsigned long auto_scavengecontrolstime = 0;

void loop_autoscavengecontrol(void)
{
  if( !auto_scavengecontrolenable ) return;
  if( millis()-auto_scavengecontrolstime<500 ) return; // limit update rate

  if( tank_full ) {
    // Stop recovery if tank is full
    if( getpump_en(RPUMPR) == RON ) setpump_en(RPUMPR, ROFF);
  } else {
    // Set recovery to delivery + 1 LPM
    if( getpump_en(RPUMPR) == ROFF ) setpump_en(RPUMPR, RON);
    setpump_lpm(RPUMPR, flow_lpm0 + 1.0f); 
  }
  
  auto_scavengecontrolstime = millis();
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
      temp_controlmode = TCNONE; // Stop auto-thermostat from overriding manual GUI intent
      htrs_enable = 1;           // Ensure state-machine allows manual heater firing
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
  FILL_PREPARE, // circulate and stabilise flow
  FILL_OVERFILL_PUMP, // pump to shower pan until tank level low
  FILL_OVERFILL_REFILL, // refill tank with inlet until level high
  FILL_DONE
};
const char *autofill_statestrs[] = {"NONE", "EMPTY", "HALF", "FULL", "PREPARE", "OF_PUMP", "OF_REFILL", "DONE", "WTF"};
int fill_overfill_count = 0;
const int fill_overfill_target = 2; // Number of repetitions

void loop_autofill(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_FILL ) return;
  auto_substatestrs = autofill_statestrs;

  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Handle substates
  switch( auto_substate ) {
    case FILL_NONE:
      if( just_entered ) {
        btLog("Entering FILL_NONE");
        auto_wbleedenable = 0;
      }
      auto_switchsubstate(FILL_FULL);
      break;

    case FILL_EMPTY:
      if( just_entered ) btLog("loop_autofill:  FILL_EMPTY not implemented");
      break;

    case FILL_HALF:  // half fill working tank from main inlet
      setrelay_en(RPINLET, RON);

      if( tank_full ) auto_switchsubstate(FILL_FULL);
      break;

    case FILL_FULL:  // full fill working tank from main inlet
      auto_wbleedenable = 1;
      setrelay_en(RPINLET, RON);

      if( tank_full ) {
        setrelay_en(RPINLET, ROFF); // Explicitly turn off the inlet valve
        auto_switchsubstate(FILL_PREPARE);
      }
      break;

    case FILL_PREPARE:
      if( just_entered ) {
        btLog("FILL: Starting circulation and stabilising flow.");
        setpump_perc(RPUMPR, 100);
        setpump_perc(RPUMPD, 50); // DELIVERY ~50%
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPR, RON);
        setpump_en(RPUMPD, RON);
        auto_wtopupenable = 1;
      }
      
      // Wait for the tank to be full AND for flow to be stable for 20s.
      if( tank_full && (millis() - flow_lastlt1 > 20000) ) {
        btLog("System is full and flow is stable. Advancing to overfill pump stage.");
        auto_switchsubstate(FILL_OVERFILL_PUMP);
      }
      break;

    case FILL_OVERFILL_PUMP:
      if( just_entered ) {
        btLog("FILL: Overfill pump stage. Run " + String(fill_overfill_count + 1) + "/" + String(fill_overfill_target) + ". SCAVENGE off, topup off, DELIVERY 90%.");
        setpump_en(RPUMPR, ROFF);
        auto_wtopupenable = 0;
        setrelay_en(RPINLET, ROFF);
        setrelay_en(RPDELIVER, RON);
        setpump_perc(RPUMPD, 90);
        setpump_en(RPUMPD, RON);
      }

      // Run until level LOW (bottom sensor level_high0 is false)
      if( tank_empty ) {
        btLog("FILL: Tank level low. Moving to refill stage.");
        auto_switchsubstate(FILL_OVERFILL_REFILL);
      }
      break;

    case FILL_OVERFILL_REFILL:
      if( just_entered ) {
        btLog("FILL: Overfill refill stage. DELIVERY and SCAVENGE off. INLET open.");
        setpump_en(RPUMPD, ROFF);
        setpump_en(RPUMPR, ROFF);
        setrelay_en(RPDELIVER, ROFF);
        setrelay_en(RPINLET, RON);
      }

      // Run until level HIGH (top sensor level_high1 is true)
      if( tank_full ) {
        fill_overfill_count++;
        btLog("FILL: Overfill refill run " + String(fill_overfill_count) + " completed.");
        if( fill_overfill_count >= fill_overfill_target ) {
          btLog("FILL: Completed all overfill cycles.");
          auto_switchsubstate(FILL_DONE);
        } else {
          auto_switchsubstate(FILL_OVERFILL_PUMP);
        }
      }
      break;

    case FILL_DONE:   
      if( auto_double ) {
        btLog("FILL_DONE so switching to STATE_WARM");
        auto_switchstate(STATE_WARM);
      } else {
        btLog("FILL_DONE so switching to STATE_SETUP1");
        auto_switchstate(STATE_SETUP1);
      }
      break;
  }

  return;
}

// Auto warm
enum autowarm_states {
  WARM_NONE = 0,
  WARM_PREPARE, // cycle water with heaters on to reach target temp
};
const char *autowarm_statestrs[] = {"NONE", "PREPARE", "WTF"};
void loop_autowarm(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_WARM ) return;
  auto_substatestrs = autowarm_statestrs;
  
  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Handle substates
  switch( auto_substate ) {
    case WARM_NONE:
      if( just_entered ) {
        btLog("Entering WARM_NONE: Preparing for shower.");
        // Use bang-bang on heaters to reach and hold target temperature.
        temp_controlmode = TCHEATER;
        auto_wtopupenable = 1; // Keep tank full
        auto_woverflowstopenable = 1; // Pause high-rate recovery when tank is full
      }
      auto_switchsubstate(WARM_PREPARE);
      break;

    case WARM_PREPARE: {
      static bool ready_logged = false;
      float base_lpm = 0.10f * modeld.maxflow; // Low flow to minimise circuit heat loss
      float pulse_lpm = 0.70f * modeld.maxflow; // Periodic pulse to guarantee mixing
      unsigned long cycle_time = (millis() - auto_substatestime) % 30000; // 30s cycle

      if( just_entered ) {
        btLog("WARM: Heating and mixing water. Target: " + String(temp_setpoint, 1) + "C");
        setrelay_en(RPDELIVER, RON);  
        setpump_en(RPUMPR, RON);  
        setpump_en(RPUMPD, RON); 
        setpump_perc(RPUMPR, 100); // Keep recovery pump high to clear pan
        ready_logged = false;
      }

      // Pulse flow for 5 seconds every 30 seconds for good mixing
      if( cycle_time < 5000 ) {
        setpump_lpm(RPUMPD, pulse_lpm);
      } else {
        setpump_lpm(RPUMPD, base_lpm);
      }

      // Once temperature reaches the run setpoint, log it, but only once.
      if (temp1 >= temp_setpoint && !ready_logged) {
        btLog("Shower is ready. Temperature is at target. Advance to WASH state when ready.");
        ready_logged = true;
      }

      // This state does not auto-advance. It holds the temperature until the user
      // manually advances to STATE_WASH. This provides convenience at the cost of
      // energy if left in this state for a long time.

      break;
    }
  }
  return;
}

// Auto wash
enum autowash_states {
  WASH_NONE = 0,
  WASH_OVERSCAVENGE,
  WASH_CYCLE
};
const char *autowash_statestrs[] = {"NONE", "OVERSCAV", "CYCLE", "WTF"};
void loop_autowash(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_WASH ) return;
  auto_substatestrs = autowash_statestrs;

  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Handle substates
  switch( auto_substate ) {
    case WASH_NONE:
      if( just_entered ) {
        btLog("Entering WASH_NONE: Starting shower.");
        // Use PID on flow and bang-bang on heaters to hold temperature.
        temp_controlmode = TCSPEED;
        auto_wtopupenable = 1; // Keep tank full
        auto_wtopup_interval = 20000; // 20s topup interval during WASH
        wash_temp_stabilised = false;
        wash_temp_stable_start_time = 0;
        // Do not enable scavenge control yet; allow overscavenge first
      }
      auto_switchsubstate(WASH_OVERSCAVENGE);
      break;

    case WASH_OVERSCAVENGE:
      if( just_entered ) {
        btLog("WASH: Overscavenging pan.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPD, RON);
        setpump_en(RPUMPR, RON);
        setpump_perc(RPUMPR, 100); // Clear pan buildup
      }
      if (millis() - auto_substatestime > 5000) {
        auto_switchsubstate(WASH_CYCLE);
      }
      break; 

    case WASH_CYCLE:
      if( just_entered ) {
        btLog("WASH: PID temperature control active.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPD, RON);
        auto_scavengecontrolenable = 1; // Now dynamically manage recovery flow
      }
      // In this state, loop_tempcontrol() is handling both the heaters (bang-bang)
      // and the delivery pump speed (PID) because temp_controlmode is TCSPEED.
      // The recovery pump is managed by loop_autoscavengecontrol().

      // Manage auto-topup based on temperature stabilization and lower level sensor
      if( !wash_temp_stabilised ) {
        // Check if temperature has stabilised within +/- 1.0 degree C of setpoint
        if( temp1 >= temp_setpoint - 1.0 && temp1 <= temp_setpoint + 1.0 ) {
          if( wash_temp_stable_start_time == 0 ) {
            wash_temp_stable_start_time = millis();
          } else if( millis() - wash_temp_stable_start_time >= 30000 ) {
            wash_temp_stabilised = true;
            auto_wtopupenable = 0; // Disable auto topup
            btLog("WASH: Temperature stabilised, disabling auto-topup.");
          }
        } else {
          wash_temp_stable_start_time = 0;
        }
      } else {
        // If temperature is stabilised, auto-topup remains disabled unless level drops below safe bottom sensor
        if( tank_empty ) {
          wash_temp_stabilised = false; // Reset stabilisation to allow topup and re-stabilise
          wash_temp_stable_start_time = 0;
          auto_wtopupenable = 1; // Re-enable auto-topup
          btLog("WASH: Water level below safe limit, re-enabling auto-topup.");
        }
      }

      // PID Debug Logging
      static unsigned long last_debug_log = 0;
      if (millis() - last_debug_log > 10000) {
        last_debug_log = millis();
        Serial.printf("WASH_DBG: Setpoint=%.2f, Temp=%.2f, PID_Out(LPM)=%.2f, Pump_PWM=%.1f%%, Htr_En=%d, Htr_Act=%d\n",
          temp_setpoint,
          tc_pidinput,
          tc_pidoutput,
          sc_setperc[RPUMPD],
          getrelay_en(RPHEATER),
          getrelay(RPHEATER));
      }

      // Foot advance: Step on drain to stop recovery flow
      // Give the system 15 seconds to initially prime the pumps and recover from overscavenging
      if (millis() - auto_statestime >= 15000) {
        // If recovery flow (sensor 1) has been low for 5 seconds
        if (millis() - flow_lastht1 > 5000) {
          btLog("Foot advance: No recovery flow detected, advancing to STATE_RINSE");
          auto_switchstate(STATE_RINSE);
        }
      }

      // Debug foot advance - now shows both flows
      static unsigned long last_fa_debug_wash = 0;
      if (millis() - last_fa_debug_wash > 2000) {
        last_fa_debug_wash = millis();
        Serial.printf("FootAdvance DBG (WASH): state_time=%lu, flow0(del)=%.2f, flow1(rec)=%.2f, rec_age=%lu\n", 
           millis() - auto_statestime, flow_lpm0, flow_lpm1, millis() - flow_lastht1);
      }
      break;
  }
  return;
}

// Auto rinse
enum autorinse_states {
  RINSE_NONE = 0,
  RINSE_PULSE_OFF,
  RINSE_PULSE_ON,
  RINSE_CYCLE
};
const char *autorinse_statestrs[] = {"NONE", "PULSE_OFF", "PULSE_ON", "CYCLE", "WTF"};
void loop_autorinse(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_RINSE ) return;
  auto_substatestrs = autorinse_statestrs;

  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Handle substates
  switch( auto_substate ) {
    case RINSE_NONE:
      if( just_entered ) {
        btLog("Entering RINSE_NONE: Starting incremental rinse.");
        setrelay_en(RPDELIVER, RON); // Keep valve open during state transition
        // Keep PID temperature control active
        temp_controlmode = TCSPEED;
        auto_wtopupenable = 1; 
        auto_wtopup_interval = 5000; // 5s topup interval for fast recovery
        // Do not enable scavenge control yet; allow overscavenge first
      }
      auto_switchsubstate(RINSE_PULSE_OFF);
      break;

    case RINSE_PULSE_OFF:
      if( just_entered ) {
        btLog("RINSE: Pulse indicator OFF. Diverting flow to drain.");
        setrelay_en(RPDRAIN, RON);     // Open drain
        setrelay_en(RPDELIVER, ROFF);  // Close delivery
        setpump_en(RPUMPD, RON);       // Keep pump running, vents via drain
        setpump_en(RPUMPR, RON);
        setpump_perc(RPUMPR, 100); // Clear pan buildup
      }
      if( millis() - auto_substatestime > 1500 ) auto_switchsubstate(RINSE_PULSE_ON);
      break;

    case RINSE_PULSE_ON:
      if( just_entered ) {
        btLog("RINSE: Pulse indicator ON. Restoring delivery.");
        setrelay_en(RPDELIVER, RON);   // Open delivery
        setrelay_en(RPDRAIN, ROFF);    // Close drain to give full flow for the ON pulse
        setpump_en(RPUMPD, RON);
        setpump_en(RPUMPR, RON);
        setpump_perc(RPUMPR, 100); // Clear pan buildup
      }
      // Give it time to clear the pan completely before beginning drain cycles
      if( millis() - auto_substatestime > 5000 ) auto_switchsubstate(RINSE_CYCLE);
      break;

    case RINSE_CYCLE:
      if( just_entered ) {
        btLog("RINSE: Periodic drain active.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPD, RON);
        auto_scavengecontrolenable = 1; // Hand control back to matching logic
      }

      // Cycle drain valve: open for 2 seconds every 20 seconds
      {
        unsigned long cycle_time = (millis() - auto_substatestime) % 20000;
        if( cycle_time < 2000 ) {
          if (getrelay_en(RPDRAIN) == ROFF) {
            btLog("RINSE: Periodic drain opening");
            setrelay_en(RPDRAIN, RON);
          }
        } else {
          if (getrelay_en(RPDRAIN) == RON) {
            btLog("RINSE: Periodic drain closing");
            setrelay_en(RPDRAIN, ROFF);
          }
        }
      }

      // Foot advance: Step on drain to stop recovery flow
      // Give the system 15 seconds to initially prime the pumps and recover from overscavenging
      if (millis() - auto_statestime >= 15000) {
        // If recovery flow (sensor 1) has been low for 5 seconds
        if (millis() - flow_lastht1 > 5000) {
          btLog("Foot advance: No recovery flow detected, advancing to STATE_PAUSE");
          auto_switchstate(STATE_PAUSE);
        }
      }

      // Debug foot advance
      static unsigned long last_fa_debug_rinse = 0;
      if (millis() - last_fa_debug_rinse > 2000) {
        last_fa_debug_rinse = millis();
        Serial.printf("FootAdvance DBG (RINSE): state_time=%lu, flow0(del)=%.2f, flow1(rec)=%.2f, rec_age=%lu\n", 
           millis() - auto_statestime, flow_lpm0, flow_lpm1, millis() - flow_lastht1);
      }
      break;
  }
  return;
}

// Auto pause
enum autopause_states {
  PAUSE_NONE = 0,
  PAUSE_OVERSCAVENGE,
  PAUSE_DONE
};
const char *autopause_statestrs[] = {"NONE", "OVERSCAV", "DONE", "WTF"};
void loop_autopause(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_PAUSE ) return;
  auto_substatestrs = autopause_statestrs;

  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Handle substates
  switch( auto_substate ) {
    case PAUSE_NONE:
      if( just_entered ) btLog("Entering PAUSE_NONE");
      auto_switchsubstate(PAUSE_OVERSCAVENGE);
      break;

    case PAUSE_OVERSCAVENGE:
      if( just_entered ) {
        btLog("PAUSE: Overscavenging pan before shutting down.");
        setpump_perc(RPUMPR, 100);
        setpump_en(RPUMPR, RON);
      }
      if( millis() - auto_substatestime > 5000 ) {
        auto_switchsubstate(PAUSE_DONE);
      }
      break;

    case PAUSE_DONE:
      if( just_entered ) {
        btLog("PAUSE: Idling.");
        setup_pins(); // Force all relays off safely
        temp_controlmode = TCNONE; 
        htrs_enable = 1; 
      }
      break;
  }
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
  static int last_substate = -1;
  if( auto_state!=STATE_SHUT ) return;
  auto_substatestrs = autoshut_statestrs;
  
  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  // Automatic transitions between substates
  switch( auto_substate ) {
    case SHUT_NONE:
      btLog("Entering SHUT_NONE");
      //auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = 0;  // disable auto circuits
      auto_switchsubstate(SHUT_DRAINTOSAFE);
      break;

    case SHUT_DRAINTOSAFE:
      if( just_entered ) {
        btLog("SHUT: Draining with high flow.");
        temp_controlmode = TCNONE; // Turn off heater control mode
        setrelay_en(RPHEATER, ROFF);
        setrelay_en(RPHEATERA, ROFF);
        setpump_perc(RPUMPR, 100);  // Recovery pump full on (100%)
        setpump_perc(RPUMPD, 100);  // Delivery pump full on (100%)
        setrelay_en(RPDELIVER, ROFF); // Delivery valve closed
      }
      setrelay_en(RPDRAIN, RON);     // Drain valve open
      setrelay_en(RPHEATER, ROFF);   // Keep heaters off
      setrelay_en(RPHEATERA, ROFF);
      
      setpump_en(RPUMPR, RON);
      if( auto_double ) setpump_en(RPUMPD, RON);
      
      // Use the more robust "empty" detection from the old STATE_EMPTY
      if( millis()-auto_substatestime>20000 && psens0<15.0 && millis()-flow_lastht0>5000 && millis()-flow_lastht1>5000 ) {
        btLog("SHUT_DRAINTOSAFE detected finish, system is empty.");
        auto_switchsubstate(SHUT_DONE); // For now, just finish. Later this might go to SHUT_RINSE.
      }

      break;

    case SHUT_DONE:
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
      if( tank_full ) auto_switchsubstate(SETUP1_INIT2);
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
      if( tank_full ) auto_switchsubstate(SETUP1_RUN);
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
      if( millis()-auto_substatestime>=autowarm1_waitperiod ) auto_switchsubstate(WARM1_CYCLE);
      break;

    case WARM1_CYCLE:
      if( auto_double ) setpump_en(RPUMPR, RON);

      setrelay_en(RPDELIVER, RON);
      setpump_en(RPUMPR, RON);
      if( auto_double ) setpump_en(RPUMPD, RON);

      btLog("Checking for warm cycle complete");
      if( millis()-auto_substatestime>=autowarm1_cycleperiod ) auto_switchsubstate(WARM1_DRAIN);
      break;

    case WARM1_DRAIN:     
      setpump_en(RPUMPR, RON);
        
      btLog("Checking for warm drain cycle complete");
      if( millis()-auto_substatestime>=autowarm1_drainperiod ) auto_switchsubstate(WARM1_WAIT);
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
      
      // Foot advance: Step on drain to stop flow
      // Give the system 10 seconds to initially prime
      if (millis() - auto_statestime >= 10000) {
        // If flow (sensor 1) has been low for 3 seconds
        if (millis() - flow_lastht1 > 3000) {
          btLog("Foot advance: No flow detected, advancing to STATE_PAUSE");
          auto_switchstate(STATE_PAUSE);
        }
      }
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
  loop_autowarm();
  loop_autowarm1();
  loop_autowash();
  loop_autowash1();
  loop_autorinse();
  // loop_autoflushempty();
  // loop_autoflushrefill();
  loop_autopause();
  loop_autoshut();
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
  // If leaving CALIBT, archive the current log before switching.
  if (auto_state == STATE_CALIBT && state != STATE_CALIBT) {
    calib_archive_log();
  }

  // Update total wash time when leaving wash
  if( auto_state==STATE_WASH && state!=STATE_WASH ) {
    unsigned long auto_totalwashtime;
    EEPROM.get(eepromaddr0 + 2, auto_totalwashtime);
    auto_totalwashtime += millis() - auto_statestime;
    EEPROM.put(eepromaddr0 + 2, auto_totalwashtime);
  }

  // New state persistent store
  auto_state = (auto_states)state;
  if( auto_state==STATE_WASH ) {
    auto_wtopup_count = 0;
  }
  if( auto_state==STATE_FILL ) {
    fill_overfill_count = 0;
  }
  auto_statestime = millis();
  btLog("Entering state " + String(auto_statestrs[auto_state]));
  byte bauto_state = (byte)auto_state;
  EEPROM.put(eepromaddr0, bauto_state);
  auto_substate = 0;

  rpinsen_reset();
  switch( auto_state ) {
    case STATE_NONE:  auto_nextstate = STATE_OFF;  break;
    case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
    case STATE_FILL:  auto_nextstate = STATE_WARM;  break;
    case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
    case STATE_WASH:  auto_nextstate = STATE_RINSE;  break;
    case STATE_RINSE: auto_nextstate = STATE_PAUSE;  break;
    case STATE_PAUSE:  auto_nextstate = STATE_SHUT;  break;
    case STATE_SHUT:  auto_nextstate = STATE_CALIBP;  break;
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
  auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = auto_scavengecontrolenable = 0;
  auto_wtopup_interval = 10000; // Reset topup interval to default
}

void auto_advancestate(void) 
{ 
  auto_switchstate(auto_nextstate);
}