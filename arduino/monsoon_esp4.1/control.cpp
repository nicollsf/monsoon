#include <vector>
#include "control.h"
#include "sensors.h"
#include "system.h"
#include "gui.h"
#include <Preferences.h>


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int htrs_enable = 1;  // default enabled
int htrs_forcedisable = 0;
int htrs_blocked = 0;
unsigned long htrblkd_stime;
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
      rpins_changed = 1;  htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1; 
    }
    return;
  }

  // Debounce
  if( htrs_blocked && millis()-htrblkd_stime>3000 ) htrs_blocked = 0;
  if( htrs_blocked ) return;

  int heater_pins[] = {RPHEATER, RPHEATERA};
  for( int i=0; i<2; i++ ) {
    int pin = heater_pins[i];
    int target_state = getrelay_en(pin);
    int current_state = getrelay(pin);

    if( target_state!=current_state ) {
      setrelay(pin, target_state);
      rpins_changed = 1;  htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;
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

  // For legacy reasons there is one relay to enable both pumps
  if( RPUMP_EN[0] || RPUMP_EN[1] ) setrelay_en(RPPUMP, RON);
  else setrelay_en(RPPUMP, ROFF);
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
float temp_setpoint = 40.0;
//float temp_reqsetpoint = temp_setpoint;

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
      if( htrs_enable ) loop_tempcontrolwithspeed();
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
  // Bang-bang control loop
  if( temp1<temp_setpoint ) {
    //Serial.println("In loop_tempcontrolwithheater: HEATERS ON");
    setrelay_en(RPHEATER, RON);
    setrelay_en(RPHEATERA, RON);
  } else {
    //Serial.println("In loop_tempcontrolwithheater: HEATERS OFF");
    setrelay_en(RPHEATER, ROFF);
    setrelay_en(RPHEATERA, ROFF);
  }
}

// ----------------------------------------------------------------------
//   PID temperature control using delivery speed
// ----------------------------------------------------------------------
#include <PID_v1.h>
double tc_pidsetpoint, tc_pidinput, tc_pidoutput;
double tc_Kp = -2, tc_Ki = 0, tc_Kd = 0;  // tuning parameters
PID myPID(&tc_pidinput, &tc_pidoutput, &tc_pidsetpoint, tc_Kp, tc_Ki, tc_Kd, DIRECT);

void setup_tempcontrolwithspeed(void)
{
  tc_pidsetpoint = temp_setpoint;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(10, 100); 
}

void loop_tempcontrolwithspeed(void)
{  
  tc_pidsetpoint = temp_setpoint;  
  if( temp1>-500.0 ) tc_pidinput = temp1;
  myPID.Compute(); 
  setpump_perc(RPUMPD, tc_pidoutput);
}
