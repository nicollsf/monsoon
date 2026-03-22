#include "system.h"

byte nwresets;  // number of warm restarts
int RPINS0_EN[8], RPINS1_EN[8];  // target values
int rpins_changed;

void setup_pins()
{
  // Shorted unused pins
  for( int i=0; i<16; i++ ) pinMode(UNUSEDPINS[i], INPUT);

  // Digital outputs for relays (high is off)
  for( int i=0; i<8; i++ ) {
    pinMode(RPINS0[i], OUTPUT);  pinMode(RPINS1[i], OUTPUT);
    digitalWrite(RPINS0[i], RELAYOFF);  digitalWrite(RPINS1[i], RELAYOFF);
    RPINS0_EN[i] = 0;  RPINS1_EN[i] = 0;
  }
  rpins_changed = 0;

  // Digital inputs for flow sensors
  for( int i=0; i<4; i++ ) pinMode(FSPINS[i], INPUT_PULLUP);

  // Level sensors
  for( int i=0; i<8; i++ ) pinMode(LSPINS[i], INPUT_PULLUP);

  // Temperature setup on TSAPIN handled by onewire

  // Heater triac
  pinMode(TRIAC_ZERO, INPUT);  // zero crossing detection (interrupt)
  pinMode(TRIAC_SCR, OUTPUT);  // triac gate control

  // Speed controller
  pinMode(SP_UND, OUTPUT);  // direction
  pinMode(SP_CLK, OUTPUT);  // pulse increment/decrement

  // Reset sense pin
  pinMode(RSTSENS, INPUT);
}


void loop_pins()
{
  rpins_changed = 0;  // will be nonzero on return if any pin changed

  // Pumps
  for ( int i=4; i<8; i++ ) {
    if( digitalRead(RPINS0[i])==RELAYOFF & RPINS0_EN[i]==1 ) {
      digitalWrite(RPINS0[i], RELAYON);
      rpins_changed = 1;
    }
    if( digitalRead(RPINS0[i])==RELAYON & RPINS0_EN[i]==0 ) {
      digitalWrite(RPINS0[i], RELAYOFF);
      rpins_changed = 1;
    }
  }

  // Valves
  for ( int i=0; i<8; i++ ) {
    if( digitalRead(RPINS1[i])==RELAYOFF & RPINS1_EN[i]==1 ) {
      digitalWrite(RPINS1[i], RELAYON);
      rpins_changed = 1;
    }
    if( digitalRead(RPINS1[i])==RELAYON & RPINS1_EN[i]==0 ) {
      digitalWrite(RPINS1[i], RELAYOFF);
      rpins_changed = 1;
    }
  }
}
