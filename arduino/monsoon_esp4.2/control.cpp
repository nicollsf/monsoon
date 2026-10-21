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

float ssr_main_duty = 1.0f; // Main 4kW SSR duty cycle: 0.0f (0%) to 1.0f (100%)

void loop_ssr_control(void)
{
  static unsigned long ssr_window_start = 0;
  unsigned long now = millis();
  const unsigned long SSR_WINDOW_MS = 1000;

  if (now - ssr_window_start >= SSR_WINDOW_MS) {
    ssr_window_start = now;
  }

  bool temp_safe = (temp1 <= htrs_maxtemp);
  bool level_safe = !tank_empty;
  bool safety_safe = htrs_enable && !htrs_forcedisable && level_safe && temp_safe;
  bool main_relay_on = (getrelay(RPHEATER) == RON);

  unsigned long on_duration = (unsigned long)(constrain(ssr_main_duty, 0.0f, 1.0f) * SSR_WINDOW_MS);

  if (main_relay_on && safety_safe && (now - ssr_window_start < on_duration) && on_duration > 0) {
    digitalWrite(SSR_MAIN_PIN, HIGH);
  } else {
    digitalWrite(SSR_MAIN_PIN, LOW);
  }
}

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
    digitalWrite(SSR_MAIN_PIN, LOW);
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
    loop_ssr_control();
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

  loop_ssr_control();

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
      if( sc_pwmtarg < sc_pwmw[i] ) sc_pwmw[i] = max(sc_pwmw[i] - 2, sc_pwmtarg); // smooth deceleration (~15% PWM/sec)
      else sc_pwmw[i] = min(sc_pwmw[i] + 2, sc_pwmtarg); // smooth acceleration (~15% PWM/sec)
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

thermalmodel active_thermal_model = {
  "shower with roof on and through main shower head",
  19.5f,   // tau_dead_sec: 19.5s pure transport delay at 4.76 LPM
  233.0f,  // tau_sec: 233s first-order time constant (R^2 = 0.9955)
  207.0f,  // ua_w_per_c: 207 W/C steady-state heat loss conductance
  48.2f,   // c_th_kj_per_c: 48.2 kJ/C effective circulating loop thermal capacity
  4.76f    // ref_flow_lpm: calibrated reference flow
};

void thermalmodel_save(const thermalmodel& m) {
  prefs.begin("therm_models", false);
  prefs.putBytes("active", &m, sizeof(thermalmodel));
  prefs.end();
  btLog("Thermal model saved: " + String(m.label));
}

void thermalmodel_load(void) {
  prefs.begin("therm_models", false);
  if (prefs.isKey("active") && prefs.getBytesLength("active") == sizeof(thermalmodel)) {
    prefs.getBytes("active", &active_thermal_model, sizeof(thermalmodel));
    Serial.printf("Sensors: Loaded thermal model '%s' (tau_dead=%.1fs, tau=%.1fs, UA=%.1f W/C)\n",
                  active_thermal_model.label, active_thermal_model.tau_dead_sec,
                  active_thermal_model.tau_sec, active_thermal_model.ua_w_per_c);
  } else {
    // Save defaults to initialize NVRAM
    prefs.putBytes("active", &active_thermal_model, sizeof(thermalmodel));
    Serial.printf("Sensors: Initialized NVRAM with calibrated thermal model '%s'\n", active_thermal_model.label);
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

float getlpmFrompwm(float targetpwm, pumpmodel m) {
  if (targetpwm <= m.offset) return 0.0f;
  if (m.a <= 0.001f || m.b <= 0.001f) {
    // Linear fallback if fit parameters uninitialized
    return constrain((targetpwm - 10.0f) * (10.0f / 90.0f), 0.0f, 10.0f);
  }
  float flow = m.a * pow(targetpwm - m.offset, m.b);
  if (flow > m.maxflow) flow = m.maxflow;
  return constrain(flow, 0.0f, 10.0f);
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

  thermalmodel_load();
}

void save_temp_setpoint(void) {
  Preferences prefs;
  prefs.begin("temp_control", false);
  prefs.putFloat("setpoint", temp_setpoint);
  prefs.end();
}

// Cases
#include <PID_v1.h>

void setup_tempcontrolwithspeed(void);
void loop_tempcontrolwithspeed(void);

// ----------------------------------------------------------------------
//   PID temperature control using delivery speed
// ----------------------------------------------------------------------
double tc_pidsetpoint, tc_pidinput, tc_pidoutput;
// Responsive tuning for 6.0 kW flow-based control:
// Kp=0.80 (+1.2 LPM per +1.5C error), Ki=0.025 (+2.25 LPM/min at +1.5C error), Kd=0.40 (rate damping)
double tc_Kp = 0.80, tc_Ki = 0.025, tc_Kd = 0.40;
PID myPID(&tc_pidinput, &tc_pidoutput, &tc_pidsetpoint, tc_Kp, tc_Ki, tc_Kd, REVERSE);

// ----------------------------------------------------------------------
//   PID temperature control using heater power (Main 4kW SSR modulation)
// ----------------------------------------------------------------------
double htr_pidsetpoint, htr_pidinput, htr_pidoutput;
// DIRECT mode: if temp1 < setpoint (error > 0), increase heat output (higher SSR duty)
// htr_Kp: 0.08 duty per deg C (320 W per deg C offset)
// htr_Ki: 0.002 duty per sec (smooth integral accumulation)
// htr_Kd: 0.4 duty per (deg C / sec) rate of change
double htr_Kp = 0.08, htr_Ki = 0.002, htr_Kd = 0.4;
PID htrPID(&htr_pidinput, &htr_pidoutput, &htr_pidsetpoint, htr_Kp, htr_Ki, htr_Kd, DIRECT);

void setup_tempcontrolwithheater(float initial_duty) {
  htr_pidsetpoint = temp_setpoint;
  if (temp1 > -500.0f) {
    htr_pidinput = temp1;
  } else {
    htr_pidinput = temp_setpoint;
  }
  
  initial_duty = constrain(initial_duty, 0.05f, 1.0f);
  htr_pidoutput = (double)initial_duty;
  ssr_main_duty = initial_duty;

  htrPID.SetMode(MANUAL);
  htrPID.SetOutputLimits(0.05, 1.0); // 5% (200W) to 100% (4000W) Main SSR duty
  htrPID.SetSampleTime(2000);        // 2000 ms sample period (smooth, slow heater loop)
  htrPID.SetMode(AUTOMATIC);

  setrelay_en(RPHEATERA, RON); // 2kW Aux kettle element provides stable base heat
  setrelay_en(RPHEATER, RON);  // 4kW Main geyser element relay closed, SSR actively modulates duty

  temp_controlmode = TCHEATER;
  temp_controlmodelast = TCHEATER;

  btLog("TCHEATER initialized: Initial duty=" + String((int)(initial_duty * 100.0f + 0.5f)) + "% (Aux 2kW ON + Main SSR)");
}

void loop_tempcontrolwithheater(void)
{
  htr_pidsetpoint = temp_setpoint;
  if (temp1 > -500.0f) {
    htr_pidinput = temp1;
  }
  htrPID.Compute();

  // Active heater power regulation: Aux ON (2kW) + Main SSR (0.05 - 1.0)
  setrelay_en(RPHEATERA, RON);
  setrelay_en(RPHEATER, RON);
  ssr_main_duty = constrain((float)htr_pidoutput, 0.05f, 1.0f);

  // Debug logging every 10 seconds
  static unsigned long last_htr_dbg = 0;
  if (millis() - last_htr_dbg >= 10000) {
    last_htr_dbg = millis();
    float total_kw = 2.0f + (4.0f * ssr_main_duty);
    Serial.printf("HTR_PID_DBG: SP=%.2f, Temp=%.2f, SSR_Duty=%.1f%%, Total=%.2f kW, LockedFlow=%.2f LPM, RecPWM=%.1f%%\n",
      temp_setpoint, temp1, ssr_main_duty * 100.0f, total_kw, wash_locked_flow, sc_setperc[RPUMPR]);
  }
}

void loop_tempcontrol(void)
{
  // Initialise if mode change
  if( temp_controlmode!=temp_controlmodelast ) {
    switch( temp_controlmode ) {
      case TCSPEED:
        setup_tempcontrolwithspeed();
        break;
      case TCHEATER:
        setup_tempcontrolwithheater(0.80f);
        break;
    }
    temp_controlmodelast = temp_controlmode;
  }
 
  // Loop
  switch( temp_controlmode ) {
    case TCOFF:
      setrelay_en(RPHEATER, ROFF);
      setrelay_en(RPHEATERA, ROFF);
      ssr_main_duty = 0.0f;
      break;
    case TCON:
      setrelay_en(RPHEATER, RON);
      setrelay_en(RPHEATERA, RON);
      ssr_main_duty = 1.0f;
      break;
    case TCHEATER:
      loop_tempcontrolwithheater();
      break;
    case TCSPEED:
      if( htrs_enable ) {
        // Full 6.0 kW power during flow-based control regime
        setrelay_en(RPHEATERA, RON);
        setrelay_en(RPHEATER, RON);
        ssr_main_duty = 1.0f;

        loop_tempcontrolwithspeed();
      }
      break;
  }
}

// ----------------------------------------------------------------------
//   Classical Cascaded Hydraulic-Headroom Power Supervisor
// ----------------------------------------------------------------------
// Outer loop: Regulates recovery pump load (PWM1) to setpoint SP_rec (72%)
// by adjusting total heater power target_kw.
// Evaluated on a 10-second sample period to ensure frequency separation
// from the inner 1.5s delivery flow temperature PID.
// ----------------------------------------------------------------------
static float power_integral_kw = 5.2f;
static float last_rec_pwm = 72.0f;

void reset_power_supervisor(float initial_kw) {
  power_integral_kw = constrain(initial_kw, 1.5f, 6.0f);
  last_rec_pwm = sc_setperc[RPUMPR];
  
  // Set initial hardware states with hysteresis
  bool aux_on = (power_integral_kw >= 3.0f);
  setrelay_en(RPHEATERA, aux_on ? RON : ROFF);
  
  float aux_kw = aux_on ? 2.0f : 0.0f;
  float main_kw = max(0.0f, power_integral_kw - aux_kw);
  ssr_main_duty = constrain(main_kw / 4.0f, 0.0f, 1.0f);
  setrelay_en(RPHEATER, (ssr_main_duty > 0.01f) ? RON : ROFF);
  
  btLog("Power supervisor reset: " + String(power_integral_kw, 1) + " kW (Main " + String((int)(ssr_main_duty * 100.0f + 0.5f)) + "% + Aux " + (aux_on ? "2kW" : "0kW") + ")");
}

void loop_main_power_supervisor(void)
{
  static unsigned long last_eval = 0;
  static unsigned long last_crit_check = 0;
  unsigned long now = millis();

  bool aux_on = (getrelay_en(RPHEATERA) == RON);
  float total_kw = (4.0f * ssr_main_duty) + (aux_on ? 2.0f : 0.0f);

  // Softstart: Hold starting power steady (4.2 kW) during initial flow ramp
  bool in_softstart = (auto_state == STATE_WASH && auto_substate == WASH_SOFTSTART);
  if (in_softstart) {
    last_eval = now;
    last_rec_pwm = sc_setperc[RPUMPR];
    return;
  }

  float rec_pwm = sc_setperc[RPUMPR];

  // 1. FAST CRITICAL SATURATION CHECK (Evaluated every 1000ms):
  // If recovery pump reaches 88%+ saturation, immediately drop power to protect pan from flooding
  if (now - last_crit_check >= 1000) {
    last_crit_check = now;
    if (rec_pwm >= 88.0f) {
      power_integral_kw = max(1.5f, power_integral_kw - 0.40f); // Fast 400W emergency step-down
    }
  }

  // 2. CLASSICAL PID SUPERVISORY UPDATE (Evaluated every 10 seconds):
  // Frequency separation: Outer power loop runs at 10s, giving the inner 1.5s flow PID
  // plenty of time to stabilize temperature before the next power adjustment.
  if (now - last_eval < 10000) return;
  float dt_sec = (last_eval == 0) ? 10.0f : ((now - last_eval) / 1000.0f);
  last_eval = now;

  // Setpoint: 72% recovery PWM. Deadband: +/- 3% (69% to 75% PWM)
  const float SP_REC_PWM = 72.0f;
  const float DEADBAND_PWM = 3.0f;

  float error_rec = 0.0f;
  if (rec_pwm > SP_REC_PWM + DEADBAND_PWM) {
    error_rec = rec_pwm - (SP_REC_PWM + DEADBAND_PWM); // Positive: recovery pump overloaded
  } else if (rec_pwm < SP_REC_PWM - DEADBAND_PWM) {
    error_rec = rec_pwm - (SP_REC_PWM - DEADBAND_PWM); // Negative: recovery pump has headroom
  }

  // Derivative Rate-of-Rise (Phase-Lead Damping):
  // Detects rapid recovery load surges (e.g. onset of overheating) before pump can saturate
  float d_rec_pwm_dt = (rec_pwm - last_rec_pwm) / dt_sec; // % PWM per second
  last_rec_pwm = rec_pwm;

  // Inhibit power increase if water temperature is already at or above setpoint
  if (temp1 >= temp_setpoint + 0.3f && error_rec < 0.0f) {
    error_rec = 0.0f;
  }

  // Thermal overtemp shedding if delivery flow PID is overwhelmed (temp > SP + 0.8C)
  float thermal_shed_kw = 0.0f;
  if (temp1 > temp_setpoint + 0.8f) {
    thermal_shed_kw = (temp1 - (temp_setpoint + 0.8f)) * 0.50f; // 0.5 kW per deg C overshoot
  }

  // Classical PID controller gains:
  // Kp: proportional reaction to recovery strain (kW per % PWM)
  // Ki: integral drift per 10-second period (kW per % PWM)
  // Kd: derivative rate-of-surge damping (kW per (%/s) rate of rise)
  const float Kp_rec = 0.035f;
  const float Ki_rec = 0.015f;
  const float Kd_rec = 0.35f; // Phase-lead damping for recovery surges

  float d_action = 0.0f;
  if (d_rec_pwm_dt > 0.30f) {
    d_action = Kd_rec * (d_rec_pwm_dt - 0.30f);
  }

  // Integral update (negative sign because higher rec_pwm requires LOWER power)
  power_integral_kw -= (Ki_rec * error_rec);
  power_integral_kw = constrain(power_integral_kw, 1.5f, 6.0f); // Bounded anti-windup

  // Total target power (P + I + D + Thermal trim)
  float target_kw = power_integral_kw - (Kp_rec * error_rec) - d_action - thermal_shed_kw;
  target_kw = constrain(target_kw, 1.5f, 6.0f);

  // 3. HARDWARE MAPPING WITH HYSTERESIS (Aux 2kW + Main SSR 0-4kW):
  // Seamless continuous modulation from 1.5 kW to 6.0 kW with zero relay chatter
  aux_on = (getrelay_en(RPHEATERA) == RON);
  if (!aux_on && target_kw >= 3.8f) {
    setrelay_en(RPHEATERA, RON);
    aux_on = true;
  } else if (aux_on && target_kw <= 2.2f) {
    setrelay_en(RPHEATERA, ROFF);
    aux_on = false;
  }

  float aux_kw = aux_on ? 2.0f : 0.0f;
  float main_kw = max(0.0f, target_kw - aux_kw);
  ssr_main_duty = constrain(main_kw / 4.0f, 0.0f, 1.0f);
  setrelay_en(RPHEATER, (ssr_main_duty > 0.01f) ? RON : ROFF);

  float new_total_kw = (4.0f * ssr_main_duty) + (aux_on ? 2.0f : 0.0f);
  btLog("Power supervisor (Classical PID): Target " + String(target_kw, 2) + " kW | Act: " + String(new_total_kw, 2) + " kW (Main " + String((int)(ssr_main_duty * 100.0f + 0.5f)) + "% + Aux " + (aux_on ? "2kW" : "0kW") + ") | Rec=" + String(rec_pwm, 0) + "% (dRec/dt=" + String(d_rec_pwm_dt, 2) + "%/s) | T=" + String(temp1, 1) + "C");
}

static float last_applied_del_flow = 3.5f;
static unsigned long last_del_slew_time = 0;

void setup_tempcontrolwithspeed(void)
{
  // Set the target temperature and current input for the PID controller.
  tc_pidsetpoint = temp_setpoint;
  if (temp1 > -500.0f) {
    tc_pidinput = temp1;
  } else {
    tc_pidinput = temp_setpoint;
  }
  
  // Bumpless initialization: Seed PID output with current established flow if >= 4.2 LPM, else 4.2 LPM
  float seed_flow = 4.2f;
  if (flow_lpm0 >= 4.2f && flow_lpm0 <= 8.5f) {
    seed_flow = flow_lpm0;
  }
  tc_pidoutput = seed_flow;
  last_applied_del_flow = seed_flow;
  last_del_slew_time = millis();

  // Force MANUAL -> AUTOMATIC transition so PID_v1 re-initializes ITerm to seed_flow
  myPID.SetMode(MANUAL);
  // Set output limits in Litres Per Minute (LPM): 3.6 to 8.5 LPM
  myPID.SetOutputLimits(3.6, 8.5); 
  // Evaluate every 2000 ms
  myPID.SetSampleTime(2000);
  myPID.SetMode(AUTOMATIC);
  wash_speed_start_time = millis();
}

void loop_tempcontrolwithspeed(void)
{  
  tc_pidsetpoint = temp_setpoint;  
  if( temp1 > -500.0 ) tc_pidinput = temp1;
  myPID.Compute(); 

  unsigned long now = millis();
  float desired_flow = (float)tc_pidoutput;

  // Startup Soft-Start (during WASH_SOFTSTART substate):
  // Rate-limited ramp: Start at 3.6 LPM and ramp smoothly towards target.
  bool in_softstart = (auto_state == STATE_WASH && auto_substate == WASH_SOFTSTART);
  if (in_softstart) {
    float max_soft_ramp = 3.6f + ((now - auto_substatestime) / 1000.0f) * 0.4f;
    desired_flow = min(desired_flow, max_soft_ramp);
  }

  // Constrain between safe bounds [3.6, 8.5] LPM
  desired_flow = constrain(desired_flow, 3.6f, 8.5f);

  // Slew rate limiter: Maximum change of +/- 0.20 LPM per second
  if (last_del_slew_time == 0 || now - last_del_slew_time > 3000) {
    last_applied_del_flow = desired_flow;
  } else {
    float dt_s = (now - last_del_slew_time) / 1000.0f;
    float max_step = max(0.02f, 0.20f * dt_s);
    if (desired_flow > last_applied_del_flow + max_step) {
      desired_flow = last_applied_del_flow + max_step;
    } else if (desired_flow < last_applied_del_flow - max_step) {
      desired_flow = last_applied_del_flow - max_step;
    }
    last_applied_del_flow = desired_flow;
  }
  last_del_slew_time = now;

  setpump_lpm(RPUMPD, desired_flow);
}
