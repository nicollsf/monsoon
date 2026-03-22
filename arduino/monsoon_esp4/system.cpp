#include "system.h"

//byte nwresets;  // number of warm restarts
//int RPINS0_EN[8], RPINS1_EN[8];  // target values
//int rpins_changed;

int RPINS_EN[6];  // target values
int rpins_changed;

void setup_pins()
{
  // Digital outputs for relays off
  for( int i=0; i<7; i++ ) {
    pinMode(RPINS[i], OUTPUT);
    RPINS_EN[i] = 0;
    digitalWrite(RPINS[i], RPINS_ROFF[i]);
  }
  rpins_changed = 0;

  // Digital inputs for flow sensors
  for( int i=0; i<2; i++ ) pinMode(FSPINS[i], INPUT_PULLUP);

  // Level sensors
  for( int i=0; i<2; i++ ) pinMode(LSPINS[i], INPUT_PULLUP);

  // Temperature setup on TSAPIN handled by onewire
  // Temperature setup on TSA2PIN set up by analogRead

//   // Heater triac
//   pinMode(TRIAC_ZERO, INPUT);  // zero crossing detection (interrupt)
//   pinMode(TRIAC_SCR, OUTPUT);  // triac gate control

  // Speed controller (BEWARE:  pins are duplicated below)
  pinMode(SC_UND, OUTPUT);  // pigipot direction
  pinMode(SC_CLK, OUTPUT);  // pigipot pulse increment/decrement
  pinMode(SC_EN, OUTPUT);  // hbridge enable
  pinMode(SC_PWM, OUTPUT);  // hbridge PWM output value

//   // Reset sense pin
//   pinMode(RSTSENS, INPUT);
}


void loop_pins()
{
  rpins_changed = 0;  // will be nonzero on return if any pin changed
  
  // Make relays follow command input
  for( int i=0; i<7; i++ ) {
    if( digitalRead(RPINS[i])==RPINS_ROFF[i] & RPINS_EN[i]==1 ) {
      digitalWrite(RPINS[i], !RPINS_ROFF[i]);
      rpins_changed = 1;
    }
    if( digitalRead(RPINS[i])==!RPINS_ROFF[i] & RPINS_EN[i]==0 ) {
      digitalWrite(RPINS[i], RPINS_ROFF[i]);
      rpins_changed = 1;
    }
  }
}
