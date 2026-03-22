#include "control.h"
#include "system.h"
#include "gui.h"


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

void setup_heatertriac()
{
  // set up zero crossing interrupt
  attachInterrupt(digitalPinToInterrupt(TRIAC_ZERO), zeroCrossingInterrupt, CHANGE);
}

// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

// Heaters and corresponding level sensor protection
int lshtrs[4] = {1, 1, 3, 3};  // level sensors protecting corresponding heaters
int htrs_disable = 0;  // override
int htrs_blocked = 0;
unsigned long htrblkd_stime;
int htrs_changed, htrs_tripped;  // check after call to function


void loop_heaters(void)
{
  htrs_changed = htrs_tripped = 0;
  
  // Safe force all off
  if( htrs_disable ) {
    for( int i=0; i<4; i++ ) {
      if( digitalRead(RPINS0[i])!=RELAYOFF) {
        digitalWrite(RPINS0[i], RELAYOFF);
        htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
      }
    }
  }

  // Safe heaters protected by level sensors
  for( int i=0; i<4; i++ ) {
    int lshtr = lshtrs[i];
    if( digitalRead(RPINS0[i])==RELAYON && digitalRead(LSPINS[lshtr])==LSPINSlv[lshtr] ) {
      btLog("HTR " + String(i) + " OFF because LEV " + String(lshtr) + " low");
      digitalWrite(RPINS0[i], RELAYOFF);
      htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
      htrs_tripped = 1;
    }
  }

  // Debounce
  if( htrs_blocked && millis()-htrblkd_stime>3000 ) htrs_blocked = 0;
  if( htrs_blocked ) return;

  // Switch off as required
  for( int i=0; i<4; i++ ) {
    if( RPINS0_EN[i]==1 ) continue;
    
    if( digitalRead(RPINS0[i])==RELAYON ) {
      btLog("HTR " + String(i) + " off");
      digitalWrite(RPINS0[i], RELAYOFF);
      htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
    }
  }

  // Switch on as required
  int nhon = 0;
  for( int i=0; i<4; i++ ) {
    if( RPINS0_EN[i]==0 ) continue;
    if( nhon++>=2 ) continue;  // limit number of heaters

    int lshtr = lshtrs[i];
    if( digitalRead(RPINS0[i])==RELAYOFF && digitalRead(LSPINS[lshtr])!=LSPINSlv[lshtr] ) {
      digitalWrite(RPINS0[i], RELAYON);
      btLog("HTR " + String(i) + " on");
      htrs_changed = 1;  htrblkd_stime = millis();  htrs_blocked = 1;  
    }
  }  
}
