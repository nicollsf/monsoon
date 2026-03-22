#include "system.h"

//byte nwresets;  // number of warm restarts
//int RPINS0_EN[8], RPINS1_EN[8];  // target values
//int rpins_changed;

int RPINS_EN[7];  // target values
//void setrelay(int rpin, bool rval) { digitalWrite(RPINS[rpin], rval==ROFF ? RPINS_ROFF[rpin] : !RPINS_ROFF[rpin]); }
int rpins_changed;
void dumprelays(void)
{
  String mstr;
  for( int i=0; i<7; i++ ) {
    if( getrelay_en(i)==ROFF ) mstr += 0;
    else mstr += 1;
  }
  mstr += " ";
  for( int i=0; i<7; i++ ) {
    if( getrelay(i)==ROFF ) mstr += 0;
    else mstr += 1;
  }
  Serial.println(mstr);
}

void setup_pins()
{
  // Digital outputs for relays
  for( int i=0; i<7; i++ ) pinMode(RPINS[i], OUTPUT);
  for( int i=0; i<7; i++ ) setrelay(i, ROFF);
  rpinsen_reset();
  rpins_changed = 0;

  // Digital inputs for flow sensors
  for( int i=0; i<2; i++ ) pinMode(FSPINS[i], INPUT_PULLUP);

  // Level sensors
  for( int i=0; i<2; i++ ) pinMode(LSPINS[i], INPUT_PULLUP);
  for( int i=0; i<2; i++ ) pinMode(LSCPINS[i], INPUT_PULLUP);

  // Temperature setup on TSAPIN handled by onewire
  // Temperature setup on TSA2PIN set up by analogRead

  //   // Heater triac
  //   pinMode(TRIAC_ZERO, INPUT);  // zero crossing detection (interrupt)
  //   pinMode(TRIAC_SCR, OUTPUT);  // triac gate control

  // Speed controller
  //pinMode(SC_UND, OUTPUT);  // pigipot direction
  //pinMode(SC_CLK, OUTPUT);  // pigipot pulse increment/decrement
  //pinMode(SC_EN, OUTPUT);  // hbridge enable
  pinMode(SC_PWM0, OUTPUT);  // hbridge PWM1 output value on 7960 board
  pinMode(SC_PWM1, OUTPUT);  // hbridge PWM2 output value

  //   // Reset sense pin
  //   pinMode(RSTSENS, INPUT);
}


void loop_pins()
{
  
  // Make relays follow command input
  for( int i=0; i<7; i++ ) {

    // Extra safe heaters
    if( (i==RPHEATER) || (i==RPHEATERA) ) {
      if( !htrs_enable || htrs_forcedisable || (digitalRead(LSPINS[0])==LSPINSlv[0]) ) {
        if( getrelay(i)==RON ) rpins_changed = 1;
        setrelay(i, ROFF);
      }
      continue;
    }
    
    // Update relays as required
    if( getrelay_en(i)!=getrelay(i) ) {
      //Serial.println("In loop_pins: calling setrelay(" + String(i) + ", " + String(getrelay_en(i)) + ")");
      setrelay(i, getrelay_en(i));
      rpins_changed = 1;
    }

  }

  return;
}
