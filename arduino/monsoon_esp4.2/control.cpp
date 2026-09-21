#include <vector>
#include "control.h"
#include "sensors.h"
#include "system.h"
#include "gui.h"
#include <Preferences.h>
#include "auto.h"
#include "calib.h"


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int htrs_enable = 1;  // default enabled
int htrs_forcedisable = 0;
int htrs_changed;  // check after call to function
float htrs_maxtemp = 62.5;
bool pump_safety_veto = false; // Veto for recovery pump due to tank full
bool inlet_safety_veto = false; // Veto for inlet valve due to tank full

void loop_heaters(void)
{
  htrs_changed = 0;

  bool temp_safe = (temp1 <= htrs_maxtemp);
  bool level_safe = !tank_empty; // Level is safe if the tank is not empty

  static unsigned long level_safe_since = 0;
  if (!level_safe) {
    level_safe_since = 0;
  } else if (level_safe_since == 0) {
    level_safe_since = millis();
  }

  // Require level to be continuously safe for at least 3000ms before re-arming heaters
  bool level_confirmed_safe = level_safe && (level_safe_since != 0) && (millis() - level_safe_since >= 3000);

  static unsigned long last_heater_switch_time = 0;

  // Force heaters off immediately if disabled or unsafe (Safety cut-out bypasses the lockout)
  if( !htrs_enable || htrs_forcedisable || !level_safe || !temp_safe ) {
    if( getrelay(RPHEATER)==RON || getrelay(RPHEATERA)==RON ) {
      String reason = "Safety Cutout: Heaters forced OFF because: ";
      if (!htrs_enable) reason += "[Not enabled by state] ";
      if (htrs_forcedisable) reason += "[Manual force disable] ";
      if (!level_safe) reason += "[Tank empty] ";
      if (!temp_safe) reason += "[Temp exceeded " + String(htrs_maxtemp, 1) + "C (current=" + String(temp1, 1) + "C)] ";
      
      btLog(reason);
      Serial.println(reason);

      setrelay(RPHEATER, ROFF);  setrelay(RPHEATERA, ROFF);
      rpins_changed = 1;  htrs_changed = 1;
      last_heater_switch_time = millis(); // update timer to prevent immediate bounce back on
    }
    return;
  }

  // Wait for both level confirmation debounce and relay anti-chatter lockout
  if (!level_confirmed_safe || (millis() - last_heater_switch_time < 3000)) {
    return;
  }

  int heater_pins[] = {RPHEATER, RPHEATERA};
  bool changed = false;
  for( int i=0; i<2; i++ ) {
    int pin = heater_pins[i];
    int target_state = getrelay_en(pin);
    int current_state = getrelay(pin);

    if( target_state!=current_state ) {
      String htr_name = (pin == RPHEATER) ? "Main Heater" : "Aux Heater";
      String state_str = (target_state == RON) ? "ON" : "OFF";
      btLog(htr_name + " turned " + state_str);
      Serial.println(htr_name + " turned " + state_str);

      setrelay(pin, target_state);
      rpins_changed = 1;  htrs_changed = 1;
      changed = true;
    }
  }

  if (changed) {
    last_heater_switch_time = millis();
  }

  return;
}

// ----------------------------------------------------------------------
//   Pumps and Valves Safety Gatekeeper
// ----------------------------------------------------------------------
void loop_pumps_and_valves(void)
{
  // If the tank is full, set safety vetoes for recovery pump and inlet valve.
  // Exception: during scavenge pump calibration or when closed-loop scavenge control is active (STATE_WARM / STATE_WASH),
  // we do not hard-cut the recovery pump to 0 PWM, allowing the closed-loop AIMD controller to throttle it gracefully.
  if (tank_full) {
    if ((auto_state == STATE_CALIBP && auto_substate == CALIBP_CALIB_SCAVENGE) || 
        auto_state == STATE_CALIBF || 
        auto_scavengecontrolenable) {
      pump_safety_veto = false;
    } else {
      pump_safety_veto = true;
    }
    inlet_safety_veto = true;
  } else {
    pump_safety_veto = false;
    inlet_safety_veto = false;
  }
}


// ----------------------------------------------------------------------
//   Speed control
// ----------------------------------------------------------------------

unsigned long sc_lastupdate = 0;  // last time any action was taken on loop
//float sc_currperc;

//#define SC_DIGIPOT
#define SC_HBRIDGE

// -------------------------------------------
//   Speed control (digipot)
// -------------------------------------------

#ifdef SC_DIGIPOT
int sc_numpos = 128;  // number of resistor states
int sc_currpos;  // current count position
int sc_setpos;  // tc_pidsetpoint position
unsigned int sc_resetcnt;  // if nonzero then resetting

//float sc_Sp = 100;  // speed tc_pidoutput percentage

// Change triggered on SC_CLK high->low
hw_timer_t *Timer0_Cfg = NULL;
void IRAM_ATTR Timer0_ISR()
{
  if( digitalRead(SC_CLK)==LOW ) {
    digitalWrite(SC_CLK, HIGH);
    if( sc_resetcnt==0 && sc_currpos==sc_setpos ) timerAlarmDisable(Timer0_Cfg);
    return;
  }

  // Handle reset sequence
  if( sc_resetcnt!=0 ) {
    sc_resetcnt--;
    if( sc_resetcnt==0 ) sc_currpos = sc_numpos - 1;  // top level
    digitalWrite(SC_CLK, LOW);
    return;
  }

  // General
  if( sc_currpos==sc_setpos ) return;
  else if( sc_currpos<sc_setpos ) {
    digitalWrite(SC_UND, HIGH);  sc_currpos++;  digitalWrite(SC_CLK, LOW); 
  }
  else if( sc_currpos>sc_setpos ) {
    digitalWrite(SC_UND, LOW);  sc_currpos--;  digitalWrite(SC_CLK, LOW);  
  }
}

void speedcontrol_set(int setperc, int resetflag=0)
{
  if( setperc<0 || setperc>100 ) {
    Serial.println("In speedcontrol_set:  setperc out of range (returning)");
    return;
  }
  sc_setperc = setperc;

  sc_setpos = setperc/100.0*(sc_numpos-1);
  if( sc_setpos<0 || sc_setpos>=sc_numpos ) {
    Serial.println("In speedcontrol_set:  setpos out of range");
    return;
  }
  
  // Reset (restart if currently resetting)
  if( sc_resetcnt!=0 || resetflag ) {
    sc_resetcnt = sc_numpos + 10;
    digitalWrite(SC_UND, HIGH);  // reset to top level
    sc_currpos = -1000;
  }

  Serial.println("In speedcontrol_set:  setting setpos=" + String(sc_setpos));
  timerAlarmEnable(Timer0_Cfg);
}

void setup_speedcontrol(void)
{
  sc_resetcnt = 0;  // if nonzero then resetting
  sc_currpos = -1000;  // only valid if not resetting
  sc_setpos = sc_numpos - 1;

  Timer0_Cfg = timerBegin(0, 8, true);  // 10M tick/s (0.1us) upwards
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100, true);  // trigger count (10us)

  // Set value with reset 
  speedcontrol_set(sc_numpos-1, 1);
}

unsigned long sc_lastreset = 0;
void loop_speedcontrol(void)
{
  //Serial.println("Speed controller: " + String(sc_Sp));  delay(20);
  if( millis()-sc_lastreset<sc_resetperiod ) return;
  sc_lastreset = millis();

  speedcontrol_set(sc_setpos, 1);  // reset
  Serial.println("Resetting: calling speedcontrol_set(sc_setpos, 1)");
}

#endif


// -------------------------------------------
//   Speed control (hbridge)
// -------------------------------------------

#ifdef SC_HBRIDGE

int RPUMP_EN[2];  // enable
float sc_setperc[2];  // desired speed values

int sc_pwmw[2];  // internal variables to permit acceleration

void setup_speedcontrol(void)
{
  analogWriteFrequency(SC_PWM0, 10000);
  analogWriteFrequency(SC_PWM1, 10000);
  sc_pwmw[0] = 0;  analogWrite(SC_PWM0, sc_pwmw[0]);
  sc_pwmw[1] = 0;  analogWrite(SC_PWM1, sc_pwmw[1]);
  pumpsen_reset();

  pumpcalib_loadmodels();
}


void loop_speedcontrol(void)
{
  if( millis()-sc_lastupdate<50 ) return;
  sc_lastupdate = millis();

  // Find new pump values
  for( int i=0; i<2; i++ ) {
    if( RPUMP_EN[i]==ROFF || (i == RPUMPR && pump_safety_veto) ) sc_pwmw[i] = 0;
    else {
      int sc_pwmtarg = 255.0/100.0*sc_setperc[i];
      if( sc_pwmtarg<=sc_pwmw[i] ) sc_pwmw[i] = sc_pwmtarg;
      else sc_pwmw[i] = min(sc_pwmw[i]+3, sc_pwmtarg);  // limit acceleration
    }
  }

  // Write analog output values
  analogWrite(SC_PWM0, sc_pwmw[0]);
  analogWrite(SC_PWM1, sc_pwmw[1]);

  // Manage the main pump power relay (hardware gatekeeper for 12V/24V pump supply)
  bool r_active = (RPUMP_EN[RPUMPR] == RON && sc_setperc[RPUMPR] > 0.0f && !pump_safety_veto);
  bool d_active = (RPUMP_EN[RPUMPD] == RON && sc_setperc[RPUMPD] > 0.0f);
  bool pumps_active = r_active || d_active;

  if( pumps_active ) {
    setrelay_en(RPPUMP, RON);
  } else if( auto_state != STATE_OFF ) {
    setrelay_en(RPPUMP, ROFF);
  }
}


#endif

// -------------------------------------------
//   Speed control (calibration)
// -------------------------------------------

pumpmodel modelr, modeld;
Preferences prefs;

void pumpcalib_savemodels() {
  prefs.begin("pump_models", false);
  prefs.putBytes("modelr", &modelr, sizeof(pumpmodel));
  prefs.putBytes("modeld", &modeld, sizeof(pumpmodel));
  prefs.end();
}

void pumpcalib_loadmodels() {
  prefs.begin("pump_models", false);
  if (prefs.isKey("modelr") && prefs.getBytesLength("modelr") == sizeof(pumpmodel)) {
    prefs.getBytes("modelr", &modelr, sizeof(pumpmodel));
    prefs.getBytes("modeld", &modeld, sizeof(pumpmodel));
  } else {
    // Default "sane" values if no calib exists
    btLog("WARNING: No pump calibration found in NVRAM! Loading default model values. Recalibration required.");
    modelr = {10.0, 0.1, 1.0, 10.0}; 
    modeld = {10.0, 0.1, 1.0, 10.0};
  }
  prefs.end();
}

pumpmodel pumpcalib_fit(const std::vector<float>& pwms, const std::vector<float>& flows) {
    float offset = 7.5; // Based on your data, flow starts between 5% and 10%
    float maxF = 0;
    
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    int n = 0;

    for (size_t i = 0; i < pwms.size(); i++) {
        if (flows[i] > maxF) maxF = flows[i];
        // Only fit data where flow is positive and PWM > offset
        if (flows[i] > 0.1 && pwms[i] > offset) {
            float x = log(pwms[i] - offset);
            float y = log(flows[i]);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
            n++;
        }
    }

    if (n < 2) return {offset, 0.1, 1.0, 10.0}; // Return sane defaults if fit fails

    float b = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    float a = exp((sumY - b * sumX) / n);

    return {offset, a, b, maxF};
}

float getpwmFromlpm(float targetlpm, pumpmodel m) {
  if (targetlpm <= 0.01) return 0;
  
  // Inverse of the power law: (Flow/a)^(1/b) + offset
  float pwm = pow((targetlpm / m.a), (1.0 / m.b)) + m.offset;
  
  return constrain(pwm, 0, 100);
}

void setpump_lpm(int rpump, float lpm) {
  // Hard safety limit on delivery pump flow rate.
  const float MAX_DELIVERY_LPM = 9.0f;
  if (rpump == RPUMPD && lpm > MAX_DELIVERY_LPM) {
    lpm = MAX_DELIVERY_LPM;
  }

  if (lpm <= 0.05) {
    setpump_perc(rpump, 0);
    return;
  }
  pumpmodel m = (rpump==RPUMPR) ? modelr : modeld;

  // Clamp to the physical max detected during calibration
  if( lpm>m.maxflow ) lpm = m.maxflow;

  // Inverse Power Law: PWM = (Flow / a)^(1 / b) + offset
  float pwm = pow((lpm / m.a), (1.0 / m.b)) + m.offset;
  
  setpump_perc(rpump, pwm); 
}


// ----------------------------------------------------------------------
//   Temperature control
// ----------------------------------------------------------------------

TCMODE temp_controlmode = TCNONE;
TCMODE temp_controlmodelast = TCNONE;
float temp_setpoint = 50.0;
unsigned long wash_speed_start_time = 0;
//float temp_reqsetpoint = temp_setpoint;

void setup_tempcontrol(void) {
  Preferences prefs;
  prefs.begin("temp_control", false);
  temp_setpoint = prefs.getFloat("setpoint", 50.0);
  prefs.end();
}

void save_temp_setpoint(void) {
  Preferences prefs;
  prefs.begin("temp_control", false);
  prefs.putFloat("setpoint", temp_setpoint);
  prefs.end();
}

// Cases
void setup_tempcontrolwithheater(void);
void loop_tempcontrolwithheater(void);
void setup_tempcontrolwithspeed(void);
void loop_tempcontrolwithspeed(void);

void loop_tempcontrol(void)
{
  // Initialise if mode change
  if( temp_controlmode!=temp_controlmodelast ) {
    switch( temp_controlmode ) {
      case TCSPEED:
        setup_tempcontrolwithspeed();
        break;
    }
    temp_controlmodelast = temp_controlmode;
  }
 
  // Loop
  switch( temp_controlmode ) {
    case TCOFF:
      setrelay_en(RPHEATER, ROFF);
      setrelay_en(RPHEATERA, ROFF);
      break;
    case TCON:
      setrelay_en(RPHEATER, RON);
      setrelay_en(RPHEATERA, RON);
      break;
    case TCHEATER:
      loop_tempcontrolwithheater();
      break;
    case TCSPEED:
      if( htrs_enable ) {
        // Multi-stage Staged Thermal Control:
        // Main heater (4kW, RPHEATER) + Aux heater (2kW, RPHEATERA) = 6kW total.
        // Default: Stage 3 (6kW, both heaters ON) for normal high-flow showers.
        // If recovery flow is restricted by a dirty filter and delivery is capped for >= 10s:
        // - Stage 2 (4kW: Main ON, Aux OFF) if temp1 > temp_setpoint + 1.0°C (matches ~4-5 LPM)
        // - Stage 1 (2kW: Main OFF, Aux ON) if temp1 > temp_setpoint + 1.8°C (matches ~2-3 LPM)
        // - Stage 0 (0kW: Both OFF) if temp1 > temp_setpoint + 2.5°C
        // Re-engage full power as water cools to <= temp_setpoint + 0.3°C or flow restriction clears.

        extern int current_power_stage;
        static unsigned long last_stage_switch_time = 0;
        unsigned long now = millis();

        extern bool delivery_flow_restricted;
        extern unsigned long flow_restricted_since;

        // Delivery flow restricted for >= 5 seconds
        bool filter_restricted_persistent = delivery_flow_restricted && (flow_restricted_since != 0) && (now - flow_restricted_since >= 5000);

        bool in_softstart = (auto_state == STATE_WASH && auto_substate == WASH_SOFTSTART);

        if (!in_softstart && (now - wash_speed_start_time > 15000) && filter_restricted_persistent) {
          if (temp1 > temp_setpoint + 2.4f && current_power_stage > 0) {
            current_power_stage = 0; // 0 kW
            last_stage_switch_time = now;
            btLog("Hydraulic Limit: Temp high (" + String(temp1, 1) + "C). Heaters Stage 0 (0 kW).");
          } else if (temp1 > temp_setpoint + 1.6f && current_power_stage > 1) {
            if (now - last_stage_switch_time >= 3000) {
              current_power_stage = 1; // 2 kW (Aux only)
              last_stage_switch_time = now;
              btLog("Hydraulic Limit: Temp high (" + String(temp1, 1) + "C). Heaters Stage 1 (2 kW).");
            }
          } else if (temp1 > temp_setpoint + 0.8f && current_power_stage > 2) {
            if (now - last_stage_switch_time >= 3000) {
              current_power_stage = 2; // 4 kW (Main only, Aux OFF)
              last_stage_switch_time = now;
              btLog("Hydraulic Limit: Flow capped. Turning OFF Aux Heater -> Stage 2 (4 kW).");
            }
          }
        }

        // Hysteresis recovery: Step back up to full power if cooled, flow restriction cleared, or during soft-start
        if (in_softstart || temp1 <= temp_setpoint + 0.3f || !filter_restricted_persistent || (now - wash_speed_start_time <= 15000)) {
          if (current_power_stage < 3 && (now - last_stage_switch_time >= 3000)) {
            current_power_stage = 3;
            last_stage_switch_time = now;
            btLog("Thermal/Flow restriction cleared. Restoring Heaters Stage 3 (6 kW).");
          }
        }

        // Apply heater intents based on active power stage
        switch (current_power_stage) {
          case 3: // 6 kW (100%)
            setrelay_en(RPHEATER, RON);
            setrelay_en(RPHEATERA, RON);
            break;
          case 2: // 4 kW (66%)
            setrelay_en(RPHEATER, RON);
            setrelay_en(RPHEATERA, ROFF);
            break;
          case 1: // 2 kW (33%)
            setrelay_en(RPHEATER, ROFF);
            setrelay_en(RPHEATERA, RON);
            break;
          case 0: // 0 kW (0%)
            setrelay_en(RPHEATER, ROFF);
            setrelay_en(RPHEATERA, ROFF);
            break;
        }

        loop_tempcontrolwithspeed();
      }
      break;
  }
}

// ----------------------------------------------------------------------
//   Bang-bang temperature control using heaters
// ----------------------------------------------------------------------

void setup_tempcontrolwithheater(void) {
  return;
}

void loop_tempcontrolwithheater(void)
{
  float target = temp_setpoint;
  
  // Boost the setpoint by 2 degrees during WARM states to build thermal inertia.
  // This compensates for the temperature drop when transitioning to WASH.
  if (auto_state == STATE_WARM || auto_state == STATE_WARM1) {
    target += 2.0;
  }

  const float TOLERANCE = 1.0; // +/- 1 degree around target
  
  // Bang-bang control with hysteresis band
  if( temp1 < target - TOLERANCE ) {
    setrelay_en(RPHEATER, RON);
    setrelay_en(RPHEATERA, RON);
  } else if ( temp1 > target + TOLERANCE ) {
    setrelay_en(RPHEATER, ROFF);
    setrelay_en(RPHEATERA, ROFF);
  }
  // If inside the deadband, do nothing to prevent relay chatter.
}

// ----------------------------------------------------------------------
//   PID temperature control using delivery speed
// ----------------------------------------------------------------------
#include <PID_v1.h>
double tc_pidsetpoint, tc_pidinput, tc_pidoutput;
double tc_Kp = 0.25, tc_Ki = 0.01, tc_Kd = 0.0;
PID myPID(&tc_pidinput, &tc_pidoutput, &tc_pidsetpoint, tc_Kp, tc_Ki, tc_Kd, REVERSE);

bool delivery_flow_restricted = false;
unsigned long flow_restricted_since = 0;
int current_power_stage = 3;
static float flow_rec_smooth = 0.0f;
static unsigned long last_delivery_calc_time = 0;

void setup_tempcontrolwithspeed(void)
{
  // Set the target temperature for the PID controller.
  tc_pidsetpoint = temp_setpoint;
  // Initialize PID output to conservative starting flow (4.5 LPM)
  tc_pidoutput = 4.5;
  // Set the operational mode to automatic.
  myPID.SetMode(AUTOMATIC);
  // Set output limits in Litres Per Minute (LPM): 3.0 to 8.5 LPM
  myPID.SetOutputLimits(3.0, 8.5); 
  // Evaluate every 1500 ms
  myPID.SetSampleTime(1500);
  wash_speed_start_time = millis();
  last_delivery_calc_time = millis();
  flow_rec_smooth = 0.0f;
  delivery_flow_restricted = false;
  flow_restricted_since = 0;
  current_power_stage = 3;
}

void loop_tempcontrolwithspeed(void)
{  
  tc_pidsetpoint = temp_setpoint;  
  if( temp1 > -500.0 ) tc_pidinput = temp1;
  myPID.Compute(); 

  unsigned long now = millis();
  float dt = (now - last_delivery_calc_time) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;
  last_delivery_calc_time = now;

  // Calibrated recovery flow in true delivery units
  float rec_flow = (flow_lpm1_est > 0.01f) ? flow_lpm1_est : (flow_lpm1 * flow_rec_scale);

  // Exponential moving average filter for measured calibrated recovery flow (tau = 2.5s)
  if (flow_rec_smooth <= 0.01f) flow_rec_smooth = rec_flow;
  else flow_rec_smooth += (rec_flow - flow_rec_smooth) * (dt / 2.5f);

  float desired_flow = (float)tc_pidoutput;

  // 1. Startup Soft-Start (during WASH_SOFTSTART substate):
  // Rate-limited ramp: Start at 4.5 LPM and ramp at max +0.5 LPM/sec towards PID target.
  bool in_softstart = (auto_state == STATE_WASH && auto_substate == WASH_SOFTSTART);
  if (in_softstart) {
    float max_soft_ramp = 4.5f + ((now - auto_substatestime) / 1000.0f) * 0.5f;
    desired_flow = min(desired_flow, max_soft_ramp);
    flow_rec_smooth = max(flow_rec_smooth, rec_flow); // Pre-seed smoothed recovery
    delivery_flow_restricted = false;
    flow_restricted_since = 0;
  } else {
    // 2. Steady-State Hydraulic Protection:
    // Only activate after running in WASH_CYCLE for at least 15 seconds (to prevent premature clamping)
    bool in_wash_cycle = (auto_state == STATE_WASH && auto_substate == WASH_CYCLE);
    bool steady_state_ready = (in_wash_cycle && (now - auto_substatestime >= 15000)) || 
                              (!in_wash_cycle && (now - wash_speed_start_time >= 20000));

    // Hydraulic Protection: If recovery pump is working hard (PWM >= 80%) and delivery exceeds recovery capacity,
    // clamp delivery flow to recovery capacity minus 0.3 LPM so the tank never empties.
    bool recovery_saturated = (sc_setperc[RPUMPR] >= 80.0f);

    if (steady_state_ready && recovery_saturated && flow_rec_smooth > 1.0f) {
      float safe_delivery_cap = max(3.0f, flow_rec_smooth - 0.3f);
      if (desired_flow > safe_delivery_cap) {
        desired_flow = safe_delivery_cap;
        if (!delivery_flow_restricted) {
          delivery_flow_restricted = true;
          flow_restricted_since = now;
        }
      } else {
        delivery_flow_restricted = false;
        flow_restricted_since = 0;
      }
    } else {
      delivery_flow_restricted = false;
      flow_restricted_since = 0;
    }
  }

  // Constrain between absolute operating bounds [3.0, 8.5] LPM
  desired_flow = constrain(desired_flow, 3.0f, 8.5f);

  setpump_lpm(RPUMPD, desired_flow);
}
