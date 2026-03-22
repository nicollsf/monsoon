#include "control.h"
#include "system.h"
#include "gui.h"


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int htrs_disable = 0;  // override
int htrs_blocked = 0;
unsigned long htrblkd_stime;
int htrs_changed, htrs_tripped;  // check after call to function


void loop_heaters(void)
{
  htrs_changed = htrs_tripped = 0;
  
  // Safe force all off
  if( htrs_disable ) {
    if( digitalRead(RPINS[5]!=RPINS_ROFF[5]) ) {
      digitalWrite(RPINS[5], RPINS_ROFF[5]);
      htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
    }
  }

  int lshtr = 0;
  if( digitalRead(RPINS[5])!=RPINS_ROFF[5] && digitalRead(LSPINS[lshtr])==LSPINSlv[lshtr] ) {
    btLog("HTR OFF because LEV " + String(lshtr) + " low");
    digitalWrite(RPINS[5], RPINS_ROFF[5]);
    htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
    htrs_tripped = 1;
  }

  // Debounce
  if( htrs_blocked && millis()-htrblkd_stime>3000 ) htrs_blocked = 0;
  if( htrs_blocked ) return;

  // Switch off as required
  if( RPINS_EN[5]==0 ) {
    if( digitalRead(RPINS[5])!=RPINS_ROFF[5] ) {
      btLog("HTR off");
      digitalWrite(RPINS[5], RPINS_ROFF[5]);
      htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;
    }
  }

  // Switch on as required
  if( RPINS_EN[5]==1 ) {
    int lshtr = 0;
    if( digitalRead(RPINS[5])==RPINS_ROFF[5] && digitalRead(LSPINS[lshtr])!=LSPINSlv[lshtr] ) {
      digitalWrite(RPINS[5], !RPINS_ROFF[5]);
      btLog("HTR on");
      htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
    }
  }

}


// ----------------------------------------------------------------------
//   Speed control
// ----------------------------------------------------------------------

float sc_setperc;
//float sc_currperc;

//#define SC_DIGIPOT
#define SC_HBRIDGE

// -------------------------------------------
//   Speed control (digipot)
// -------------------------------------------

#ifdef SC_DIGIPOT
int sc_numpos = 128;  // number of resistor states
int sc_currpos;  // current count position
int sc_setpos;  // setpoint position
unsigned int sc_resetcnt;  // if nonzero then resetting

//float sc_Sp = 100;  // speed output percentage

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
void speedcontrol_set(int setperc, int resetflag=0)
{
  sc_setperc = setperc;
  if( setperc<0 ) sc_setperc = 0.0;
  if( setperc>100 ) sc_setperc = 100;
  Serial.println("In speedcontrol_set:  setting sc_setperc=" + String(sc_setperc));
}

void setup_speedcontrol(void)
{
  digitalWrite(SC_EN, HIGH);  // I'd like to tie this high and free the pin
  speedcontrol_set(100);
}

void loop_speedcontrol(void) 
{
  int pwmval = 255*sc_setperc/100;
  analogWrite(SC_PWM, pwmval);
}

#endif