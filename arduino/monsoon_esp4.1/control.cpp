#include <vector>
#include "control.h"
#include "sensors.h"
#include "system.h"
#include "gui.h"
#include <Preferences.h>
#include "auto.h"


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int htrs_enable = 1;  // default enabled
int htrs_forcedisable = 0;
int htrs_changed;  // check after call to function
float htrs_maxtemp = 62.5;

void loop_heaters(void)
{
  htrs_changed = 0;

  bool temp_safe = (temp1 <= htrs_maxtemp);
  bool level_safe = (digitalRead(LSPINS[0])!=LSPINSlv[0]);

  // Force heaters off if disabled or unsafe
  if( !htrs_enable || htrs_forcedisable || !level_safe || !temp_safe ) {
    if( getrelay(RPHEATER)==RON || getrelay(RPHEATERA)==RON ) {
      setrelay(RPHEATER, ROFF);  setrelay(RPHEATERA, ROFF);
      rpins_changed = 1;  htrs_changed = 1;
    }
    return;
  }

  int heater_pins[] = {RPHEATER, RPHEATERA};
  for( int i=0; i<2; i++ ) {
    int pin = heater_pins[i];
    int target_state = getrelay_en(pin);
    int current_state = getrelay(pin);

    if( target_state!=current_state ) {
      setrelay(pin, target_state);
      rpins_changed = 1;  htrs_changed = 1;
    }
  }

  return;
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
    if( RPUMP_EN[i]==ROFF ) sc_pwmw[i] = 0;  
    else {
      int sc_pwmtarg = 255.0/100.0*sc_setperc[i];
      if( sc_pwmtarg<=sc_pwmw[i] ) sc_pwmw[i] = sc_pwmtarg;
      else sc_pwmw[i] = min(sc_pwmw[i]+3, sc_pwmtarg);  // limit acceleration
    }
  }

  // Write analog output values
  analogWrite(SC_PWM0, sc_pwmw[0]);
  analogWrite(SC_PWM1, sc_pwmw[1]);

  // Manage the main pump power relay
  static bool pumps_were_active = false;
  static int last_rpins_reset_cnt = 0;
  bool pumps_active = (RPUMP_EN[0] == RON && sc_setperc[0] > 0.0f) || 
                      (RPUMP_EN[1] == RON && sc_setperc[1] > 0.0f);
                      
  bool system_was_reset = (rpins_reset_cnt != last_rpins_reset_cnt);
  last_rpins_reset_cnt = rpins_reset_cnt;

  // Edge-triggering allows manual control to override without constant interference
  if( (pumps_active && !pumps_were_active) || (pumps_active && system_was_reset) ) {
    setrelay_en(RPPUMP, RON);
  } else if( (!pumps_active && pumps_were_active) || (!pumps_active && system_was_reset) ) {
    setrelay_en(RPPUMP, ROFF);
  }
  pumps_were_active = pumps_active;
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
  prefs.begin("pump_models", true);
  if (prefs.getBytesLength("modelr") == sizeof(pumpmodel)) {
    prefs.getBytes("modelr", &modelr, sizeof(pumpmodel));
    prefs.getBytes("modeld", &modeld, sizeof(pumpmodel));
  } else {
    // Default "sane" values if no calib exists
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
  const float MAX_DELIVERY_LPM = 6.0f;
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
//float temp_reqsetpoint = temp_setpoint;

void setup_tempcontrol(void) {
  Preferences prefs;
  prefs.begin("temp_control", true);
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
        // In TCSPEED mode, heaters are on by default, and PID controls flow.
        // The bang-bang controller is disabled to prevent interference.
        setrelay_en(RPHEATER, RON);
        setrelay_en(RPHEATERA, RON);
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

  const float TOLERANCE = 2.0; // +/- 2 degrees around target
  
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
// PID tuning parameters. Based on system response data from CALIBT.
// Controller is DIRECT, but Kp is negative. An increase in temp (input) causes a more negative
// error, which when multiplied by a negative Kp, increases the output (flow) to cool the system.
// The Integral (I) term is necessary to eliminate steady-state error.
// The Derivative (D) term is set to zero as it can amplify sensor noise in a slow thermal
// system, causing erratic pump behavior.
// NOTE: The previous implementation used DIRECT mode with negative gains. This is non-standard and
// was causing the integral term to wind-up incorrectly, pinning the output at the minimum.
// The correct implementation for this system (where a negative error results in an increased
// output) is REVERSE mode with positive gains.
// Detuned for a system with ~30s thermal dead time.
double tc_Kp = 0.5, tc_Ki = 0.02, tc_Kd = 0.0;
PID myPID(&tc_pidinput, &tc_pidoutput, &tc_pidsetpoint, tc_Kp, tc_Ki, tc_Kd, REVERSE);

void setup_tempcontrolwithspeed(void)
{
  // Set the target temperature for the PID controller.
  tc_pidsetpoint = temp_setpoint;
  // Set the operational mode to automatic.
  myPID.SetMode(AUTOMATIC);
  // Set the output limits in Litres Per Minute (LPM).
  // Constrained to 3-6 LPM to prevent extreme over-correction and thermal shock.
  myPID.SetOutputLimits(3.0, 6.0); 
  // Evaluate less frequently (every 2 seconds) to better match the thermal inertia
  // of the water mass and prevent rapid micro-adjustments.
  myPID.SetSampleTime(2000);
}

void loop_tempcontrolwithspeed(void)
{  
  tc_pidsetpoint = temp_setpoint;  
  if( temp1>-500.0 ) tc_pidinput = temp1;
  myPID.Compute(); 
  setpump_lpm(RPUMPD, tc_pidoutput);
}
