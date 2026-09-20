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
const char *auto_statestrs[] = {"NONE", "OFF", "FILL", "SETUP1", "WARM", "WARM1", "WASH", "WASH1", "RINSE", "FLUSHE", "FLUSHR", "PAUSE", "SHUT", "CALIBP", "CALIBT", "CALIBF", "CALIBDUMP", "WTF"};
auto_states auto_state = STATE_NONE;
auto_states auto_nextstate = STATE_OFF;
const char *autonone_statestrs[] = {"NONE", "WTF"};
const char **auto_substatestrs = autonone_statestrs;
int auto_substate = 0;
//char *auto_substatestr;
unsigned long auto_statestime, auto_substatestime;

// Statistics tracking variables
static float wash_temp_sum = 0.0f;
static unsigned long wash_temp_count = 0;
static float wash_temp_max = 0.0f;
static float wash_flow_sum = 0.0f;
static unsigned long wash_flow_count = 0;
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
  if( !tank_full && (millis() - auto_wtopuplasthightime > auto_wtopup_interval) ) {
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


// Auto scavenge control with dynamic delta ramping in calibrated delivery flow units
int auto_scavengecontrolenable = 0;
unsigned long auto_scavengecontrolstime = 0;
float auto_scavenge_integral = 0.0f;
float scavenge_delta_lpm = 0.5f;
float auto_scavenge_target_lpm = 0.0f;
unsigned long scavenge_tank_full_last_time = 0;

void loop_autoscavengecontrol(void)
{
  if( !auto_scavengecontrolenable ) {
    auto_scavenge_integral = 0.0f;
    scavenge_delta_lpm = 0.5f;
    auto_scavenge_target_lpm = 0.0f;
    return;
  }
  if( millis()-auto_scavengecontrolstime < 500 ) return; // limit update rate to 500ms
  auto_scavengecontrolstime = millis();

  if( getpump_en(RPUMPR) == ROFF ) setpump_en(RPUMPR, RON);

  unsigned long now = millis();

  // TCP AIMD Scavenge Level Probing:
  // 1. Congestion Signal (Top float switch triggered -> tank is 100% full):
  //    - Gently back off recovery flow to slightly below delivery (Q_del - 0.4 LPM)
  //      so the water level drops smoothly below the top switch without hard pump shutdown.
  //    - Reset positive integral windup.
  // 2. Post-Full Stabilization Latch:
  //    - Hold delta at 0.0 LPM for 5 seconds after tank_full clears to clear float switch hysteresis.
  // 3. Additive Increase:
  //    - Gently ramp delta up (+0.005 LPM per 500ms = +0.01 LPM/sec = +0.60 LPM/min, capped at +1.5 LPM)
  //      to drain the shower pan and slowly climb back toward the top switch for the next level probe.
  if (tank_full) {
    scavenge_tank_full_last_time = now;
    scavenge_delta_lpm = -0.4f; // Controlled back-off below delivery
    if (auto_scavenge_integral > 0.0f) auto_scavenge_integral = 0.0f;
  } else if (now - scavenge_tank_full_last_time < 5000) {
    scavenge_delta_lpm = 0.0f; // 5s hysteresis stabilization
  } else if (flow_lpm0 >= 1.0f) {
    scavenge_delta_lpm = min(1.5f, scavenge_delta_lpm + 0.005f); // Additive Increase
  } else {
    scavenge_delta_lpm = 0.2f;
    auto_scavenge_integral = 0.0f;
  }

  // Desired recovery flow in standardized TRUE delivery units (LPM)
  float target_rec_lpm = max(1.5f, flow_lpm0 + scavenge_delta_lpm);
  auto_scavenge_target_lpm = target_rec_lpm;

  // 1. Feedforward baseline: Direct calibrated recovery PWM lookup from true delivery-unit target flow
  float ff_pwm = get_recovery_pwm_from_flow(target_rec_lpm);

  // 2. Closed-Loop Feedback Trimming in standardized Delivery Flow Units:
  // Only integrate when delivery flow is established (> 1.5 LPM) and state has run > 5s
  if (flow_lpm0 >= 1.5f && (now - auto_statestime > 5000)) {
    // Actual recovery flow in delivery units (flow_lpm1_est)
    float actual_rec_lpm = (flow_lpm1_est > 0.01f) ? flow_lpm1_est : (flow_lpm1 * flow_rec_scale);
    float err = target_rec_lpm - actual_rec_lpm; // positive if actual recovery is slower than target
    // Step integral by 0.30% PWM per LPM error per 500ms cycle
    float delta_i = err * 0.30f;
    
    // Anti-windup: Only integrate positive error if recovery flow has NOT yet reached delivery flow (flow_lpm0).
    // If actual recovery is already matching or exceeding delivery flow, the pan is clear and additional delta
    // is handled purely by feedforward (ff_pwm) without winding up the integral term.
    if (delta_i > 0.0f && actual_rec_lpm >= flow_lpm0) {
      delta_i = 0.0f;
    }

    // Anti-windup bounded trim between -15% and +15% PWM
    auto_scavenge_integral = constrain(auto_scavenge_integral + delta_i, -15.0f, 15.0f);
  } else {
    auto_scavenge_integral = 0.0f;
  }

  float total_pwm = constrain(ff_pwm + auto_scavenge_integral, 0.0f, 100.0f);
  setpump_perc(RPUMPR, total_pwm);
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
unsigned long ball_valve_off_start = 0;
const unsigned long BALL_VALVE_REOPEN_DELAY_MS = 300000; // 5 minutes in ms

void loop_autooff(void)
{
  if( auto_state!=STATE_OFF ) return;
  auto_substatestrs = autooff_statestrs;

  // Reopen ball valve only after 5 minutes of continuous OFF state
  if( getrelay(RPBALLVALVE) == RON ) {
    if( millis() - ball_valve_off_start >= BALL_VALVE_REOPEN_DELAY_MS ) {
      setrelay_en(RPBALLVALVE, ROFF);
      setrelay(RPBALLVALVE, ROFF);
      btLog("Ball Valve: Reopened after 5 minutes in state OFF.");
      report_valve_status();
      report_rpins();
    }
  }

  // Handle substates
  switch( auto_substate ) {
    case OFF_NONE:
      auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = auto_scavengecontrolenable = 0;
      auto_wbleedenable = 1;  // safe pressure when under manual control
      temp_controlmode = TCNONE; // Stop auto-thermostat from overriding manual GUI intent
      htrs_enable = 1;           // Ensure state-machine allows manual heater firing
      for( int i=0; i<7; i++ ) setrelay(i, ROFF);
      pumpsen_reset();
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
const int fill_overfill_target = 1; // Number of repetitions
float fill_overfill_volume_limit = 2.5f; // Target volume in Liters to pump into the pan
static float fill_overfill_volume_pumped = 0.0f;
static unsigned long last_integration_time = 0;

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
        auto_scavengecontrolenable = 0;
        setpump_perc(RPUMPR, 100);
        setpump_perc(RPUMPD, 40); // DELIVERY ~40%
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
        btLog("FILL: Overfill pump stage. Run " + String(fill_overfill_count + 1) + "/" + String(fill_overfill_target) + ". Target: " + String(fill_overfill_volume_limit, 1) + "L at 8 LPM.");
        auto_scavengecontrolenable = 0;
        setpump_en(RPUMPR, ROFF);
        auto_wtopupenable = 0;
        setrelay_en(RPINLET, ROFF);
        setrelay_en(RPDELIVER, RON);
        setpump_lpm(RPUMPD, 8.0f);
        setpump_en(RPUMPD, RON);
        
        fill_overfill_volume_pumped = 0.0f;
        last_integration_time = millis();
      }

      // Integrate flow rate to estimate volume pumped
      {
        unsigned long now = millis();
        unsigned long dt_ms = now - last_integration_time;
        last_integration_time = now;
        if (flow_lpm0 > 0.05f) {
          fill_overfill_volume_pumped += flow_lpm0 * (dt_ms / 60000.0f);
        }

        // Periodically print the progress
        static unsigned long last_of_print = 0;
        if (now - last_of_print > 2000) {
          last_of_print = now;
          Serial.printf("FILL_OVERFILL_PUMP: Pumped %.2fL / %.1fL\n", fill_overfill_volume_pumped, fill_overfill_volume_limit);
        }

        // Run until target volume reached, or safety tank_empty fallback
        if (fill_overfill_volume_pumped >= fill_overfill_volume_limit || tank_empty) {
          btLog("FILL: Overfill volume reached (" + String(fill_overfill_volume_pumped, 2) + "L). Moving to refill stage.");
          auto_switchsubstate(FILL_OVERFILL_REFILL);
        }
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
// Auto warm
const char *autowarm_statestrs[] = {"NONE", "RAMP", "HOLD", "WTF"};
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
        btLog("Entering WARM: Preparing shower circulation.");
        temp_controlmode = TCNONE; // loop_autowarm manages staged heaters directly
        auto_wtopupenable = 1; // Keep tank full
        auto_woverflowstopenable = 0; // Use smooth closed-loop scavenge control without hard cycling
        auto_scavengecontrolenable = 1; // Dynamic recovery delta tracking active
      }
      auto_switchsubstate(WARM_RAMP);
      break;

    case WARM_RAMP: {
      if( just_entered ) {
        btLog("WARM_RAMP: Fast heat-up with 6 kW at 3.8 LPM. Target: " + String(temp_setpoint, 1) + "C");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPR, RON);
        setpump_en(RPUMPD, RON);
        setpump_lpm(RPUMPD, 3.8f);
        auto_scavengecontrolenable = 1;
        // Engage full 6 kW (Main 4kW + Aux 2kW)
        setrelay_en(RPHEATER, RON);
        setrelay_en(RPHEATERA, RON);
      }

      // Fast heat-up completion: when within 0.8C of setpoint, advance to WARM_HOLD
      if (temp1 >= temp_setpoint - 0.8f && (millis() - auto_substatestime > 5000)) {
        btLog("WARM_RAMP: Target temperature reached (" + String(temp1, 1) + "C). Entering steady WARM_HOLD soak.");
        auto_switchsubstate(WARM_HOLD);
      }
      break;
    }

    case WARM_HOLD: {
      static bool ready_logged = false;
      static unsigned long last_stage_eval = 0;

      if( just_entered ) {
        btLog("WARM_HOLD: Thermal soak at 2.5 LPM. Maintaining " + String(temp_setpoint, 1) + "C.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPR, RON);
        setpump_en(RPUMPD, RON);
        setpump_lpm(RPUMPD, 2.5f);
        auto_scavengecontrolenable = 1;
        ready_logged = false;
        last_stage_eval = 0;
      }

      // Modulate staged heaters with hysteresis to maintain tight temperature equilibrium
      if (millis() - last_stage_eval >= 1000) {
        last_stage_eval = millis();

        if (temp1 < temp_setpoint - 1.2f) {
          // Stage 3 (6 kW: Main ON, Aux ON)
          setrelay_en(RPHEATER, RON);
          setrelay_en(RPHEATERA, RON);
        } else if (temp1 < temp_setpoint - 0.4f) {
          // Stage 2 (4 kW: Main ON, Aux OFF)
          setrelay_en(RPHEATER, RON);
          setrelay_en(RPHEATERA, ROFF);
        } else if (temp1 < temp_setpoint + 0.3f) {
          // Stage 1 (2 kW: Main OFF, Aux ON) - matches environmental heat loss perfectly
          setrelay_en(RPHEATER, ROFF);
          setrelay_en(RPHEATERA, RON);
        } else {
          // Stage 0 (0 kW: Both OFF)
          setrelay_en(RPHEATER, ROFF);
          setrelay_en(RPHEATERA, ROFF);
        }
      }

      // Notify user when ready for wash
      if (temp1 >= temp_setpoint && !ready_logged) {
        btLog("Shower is HOT and READY at " + String(temp1, 1) + "C! Advance to WASH when ready.");
        ready_logged = true;
      }
      break;
    }
  }
  return;
}

// Auto wash
const char *autowash_statestrs[] = {"NONE", "SOFTSTART", "CYCLE", "WTF"};
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
        temp_controlmode = TCSPEED;
        auto_wtopupenable = 0; // Never enable auto topup during wash
      }
      auto_switchsubstate(WASH_SOFTSTART);
      break;

    case WASH_SOFTSTART:
      if( just_entered ) {
        btLog("WASH: Soft-start circulation ramping.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPD, RON);
        setpump_en(RPUMPR, RON);
        auto_scavengecontrolenable = 1; // Dynamic closed-loop scavenge active
        temp_controlmode = TCSPEED;
      }

      // Exit Criteria: Advance to WASH_CYCLE when return flow is established (>= 3.0 LPM for >= 4s) or after 15s timeout
      if ((flow_lpm1 >= 3.0f && millis() - auto_substatestime >= 4000) || (millis() - auto_substatestime >= 15000)) {
        btLog("WASH: Soft-start complete (flow1=" + String(flow_lpm1, 1) + " LPM). Entering steady-state WASH_CYCLE.");
        auto_switchsubstate(WASH_CYCLE);
      }
      break;

    case WASH_CYCLE:
      if( just_entered ) {
        btLog("WASH: PID temperature control active.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPD, RON);
        auto_scavengecontrolenable = 1; // Keep closed-loop recovery tracking active
      }
      // Accumulate wash stats every second
      static unsigned long last_stat_accum = 0;
      if (millis() - last_stat_accum >= 1000) {
        last_stat_accum = millis();
        wash_temp_sum += temp1;
        if (temp1 > wash_temp_max) {
          wash_temp_max = temp1;
        }
        wash_temp_count++;
        
        wash_flow_sum += flow_lpm0;
        wash_flow_count++;
      }

      // PID Debug Logging
      static unsigned long last_debug_log = 0;
      if (millis() - last_debug_log > 10000) {
        last_debug_log = millis();
        Serial.printf("WASH_DBG: Setpoint=%.2f, Temp=%.2f, PID_Out(LPM)=%.2f, Pump_PWM=%.1f%%, Htr_En=%d/%d, Htr_Act=%d/%d\n",
          temp_setpoint,
          tc_pidinput,
          tc_pidoutput,
          sc_setperc[RPUMPD],
          getrelay_en(RPHEATER),
          getrelay_en(RPHEATERA),
          getrelay(RPHEATER),
          getrelay(RPHEATERA));
      }

      // Foot advance: Step on drain to stop recovery flow
      if (millis() - auto_statestime >= 15000) {
        // If recovery flow (sensor 1) has been low for 5 seconds
        if (millis() - flow_lastht1 > 5000) {
          btLog("Foot advance: No recovery flow detected, advancing to STATE_PAUSE");
          auto_switchstate(STATE_PAUSE, "Foot advance");
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
      if (millis() - auto_statestime >= 15000) {
        // If recovery flow (sensor 1) has been low for 5 seconds
        if (millis() - flow_lastht1 > 5000) {
          btLog("Foot advance: No recovery flow detected, advancing to STATE_PAUSE");
          auto_switchstate(STATE_PAUSE, "Foot advance");
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
        // Safely turn off pumps, heaters, and water circuits, but keep RPBALLVALVE closed
        setrelay_en(RPINLET, ROFF);
        setrelay_en(RPDRAIN, ROFF);
        setrelay_en(RPDELIVER, ROFF);
        setrelay_en(RPBURP, ROFF);
        setrelay_en(RPPUMP, ROFF);
        setrelay_en(RPHEATER, ROFF);
        setrelay_en(RPHEATERA, ROFF);
        setpump_en(RPUMPR, ROFF);
        setpump_en(RPUMPD, ROFF);
        setrelay(RPINLET, ROFF);
        setrelay(RPDRAIN, ROFF);
        setrelay(RPDELIVER, ROFF);
        setrelay(RPBURP, ROFF);
        setrelay(RPPUMP, ROFF);
        setrelay(RPHEATER, ROFF);
        setrelay(RPHEATERA, ROFF);
        temp_controlmode = TCNONE; 
        htrs_enable = 0; // Explicitly disable heaters during PAUSE
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
      
      // Foot advance: Step on drain to stop flow (Commented out/Disabled)
      /*
      if (millis() - auto_statestime >= 10000) {
        // If flow (sensor 1) has been low for 3 seconds
        if (millis() - flow_lastht1 > 3000) {
          btLog("Foot advance: No flow detected, advancing to STATE_PAUSE");
          auto_switchstate(STATE_PAUSE);
        }
      }
      */
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

  // Initialize NVS EEPROM
  EEPROM.begin(64);
  unsigned long auto_totalwashtime = 0;
  EEPROM.get(eepromaddr0 + 2, auto_totalwashtime);
  Serial.printf("NVS Setup: Persistent total wash time loaded = %.1f minutes\n", auto_totalwashtime / 60000.0f);
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
  int old_substate = auto_substate;
  auto_substate = substate;
  const char *auto_substatestr = auto_substatestrs[auto_substate];  // handler must set auto_substatestrs

  btLog("Entering substate " + String(auto_substatestr));
  auto_substatestime = millis();

  // Don't wipe pump/relay intents when transitioning between active running substates (WASH or WARM)
  if (!(auto_state == STATE_WASH && old_substate == WASH_SOFTSTART && substate == WASH_CYCLE) &&
      !(auto_state == STATE_WARM && old_substate == WARM_RAMP && substate == WARM_HOLD)) {
    auto_scavenge_integral = 0.0f;
    rpinsen_reset();
    pumpsen_reset();
  }
  
  if( auto_double && auto_state != STATE_OFF ) setrelay_en(RPBURP, RON);

  // Call auto with substatechange flag
  loop_auto();
}

void auto_switchstate(int state, String reason)
{  
  // If leaving CALIBT, archive the current log before switching.
  if (auto_state == STATE_CALIBT && state != STATE_CALIBT) {
    calib_archive_log();
  }

  // Update total wash time when leaving wash
  if( auto_state==STATE_WASH && state!=STATE_WASH ) {
    unsigned long auto_totalwashtime = 0;
    EEPROM.get(eepromaddr0 + 2, auto_totalwashtime);
    unsigned long session_duration = millis() - auto_statestime;
    auto_totalwashtime += session_duration;
    EEPROM.put(eepromaddr0 + 2, auto_totalwashtime);
    EEPROM.commit(); // Persist changes to NVS flash!

    float session_mins = session_duration / 60000.0f;
    float avg_temp = (wash_temp_count > 0) ? (wash_temp_sum / wash_temp_count) : 0.0f;
    float avg_flow = (wash_flow_count > 0) ? (wash_flow_sum / wash_flow_count) : 0.0f;
    float lifetime_mins = auto_totalwashtime / 60000.0f;

    String summary = "SHOWER_SUMMARY: duration=" + String(session_mins, 2) + "min, avg_temp=" + String(avg_temp, 1) + "C, max_temp=" + String(wash_temp_max, 1) + "C, avg_flow=" + String(avg_flow, 1) + "LPM, lifetime=" + String(lifetime_mins, 1) + "min";
    Serial.println("\n" + summary + "\n");
    btLog("Shower summary: " + String(session_mins, 1) + " min, avg " + String(avg_temp, 1) + "C");

    // Publish to the MQTT logger (sent as outlog which bypasses telemetry silence)
    extern void mqtt_log(String slog);
    mqtt_log(summary);
  }

  // New state persistent store
  auto_states old_state = auto_state;
  auto_state = (auto_states)state;
  if( auto_state==STATE_WASH ) {
    auto_wtopup_count = 0;
    // Reset wash session counters
    wash_temp_sum = 0.0f;
    wash_temp_count = 0;
    wash_temp_max = 0.0f;
    wash_flow_sum = 0.0f;
    wash_flow_count = 0;
  }
  if( auto_state==STATE_FILL ) {
    fill_overfill_count = 0;
  }
  auto_statestime = millis();
  
  String logMsg = "Entering state " + String(auto_statestrs[auto_state]);
  if (reason.length() > 0) {
    logMsg += " (Reason: " + reason + ")";
  }
  btLog(logMsg);
  
  byte bauto_state = (byte)auto_state;
  EEPROM.put(eepromaddr0, bauto_state);
  EEPROM.commit(); // Persist state change to NVS flash!
  auto_substate = 0;

  rpinsen_reset();
  if (auto_state != STATE_OFF && auto_state != STATE_NONE) {
    // Keep ball valve closed (RON) throughout any active state
    setrelay_en(RPBALLVALVE, RON);
    setrelay(RPBALLVALVE, RON);
  } else if (auto_state == STATE_OFF) {
    if (old_state != STATE_OFF) {
      // Entering state OFF: start the 5-minute timer before reopening the ball valve
      ball_valve_off_start = millis();
      btLog("State OFF: Ball valve will remain closed for 5 minutes before reopening.");
    }
  } else {
    // State NONE (boot)
    setrelay_en(RPBALLVALVE, ROFF);
    setrelay(RPBALLVALVE, ROFF);
  }

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
      auto_nextstate = STATE_CALIBF;
      // Enforce default intent for CALIBT exactly once on transition
      setpump_perc(RPUMPR, 100);
      setpump_en(RPUMPR, RON);
      break;
    case STATE_CALIBF:     auto_nextstate = STATE_CALIBDUMP; break;
    case STATE_CALIBDUMP:  auto_nextstate = STATE_OFF;       break;
        
    case STATE_SETUP1:  auto_nextstate = STATE_WARM1;  break;
    case STATE_WARM1:  auto_nextstate = STATE_WASH1;  break;
    case STATE_WASH1:  auto_nextstate = STATE_PAUSE;  break;
  }

  // Disable auto circuits
  auto_woverflowstopenable = auto_woverflowopenenable = auto_wtopupenable = auto_wburpenable = auto_wbleedenable = auto_scavengecontrolenable = 0;
  auto_scavenge_integral = 0.0f;
  auto_wtopup_interval = 10000; // Reset topup interval to default
}

void auto_advancestate(void) 
{ 
  auto_switchstate(auto_nextstate);
}