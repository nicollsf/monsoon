#include "system.h"
#include "control.h"

//byte nwresets;  // number of warm restarts
//int RPINS0_EN[8], RPINS1_EN[8];  // target values
//int rpins_changed;

int RPINS_EN[7];  // target values
int rpins_reset_cnt = 0;
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

  // Digital inputs for flow sensors (Pins 36, 39 are input-only, no internal pull-ups)
  for( int i=0; i<2; i++ ) pinMode(FSPINS[i], INPUT);

  // Level sensors
  for( int i=0; i<2; i++ ) {
    if (LSPINS[i] >= 34 && LSPINS[i] <= 39) {
      pinMode(LSPINS[i], INPUT);
    } else {
      pinMode(LSPINS[i], INPUT_PULLUP);
    }
  }
  for( int i=0; i<2; i++ ) {
    // Safely skip pull-ups for any input-only pins (34-39)
    if (LSCPINS[i] >= 34 && LSCPINS[i] <= 39) {
      pinMode(LSCPINS[i], INPUT);
    } else {
      pinMode(LSCPINS[i], INPUT_PULLUP);
    }
  }

  // Temperature setup on TSAPIN handled by onewire
  // Temperature setup on TSA2PIN set up by analogRead

  //   // Heater triac
  //   pinMode(TRIAC_ZERO, INPUT);  // zero crossing detection (interrupt)
  //   pinMode(TRIAC_SCR, OUTPUT);  // triac gate control

  // Speed controller
  //pinMode(SC_UND, OUTPUT);  // Digipot direction
  //pinMode(SC_CLK, OUTPUT);  // Digipot pulse increment/decrement
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
    
    // Update non-heater relays as required. Heaters are handled exclusively by loop_heaters().
    if( i != RPHEATER && i != RPHEATERA) {
      int target_state = getrelay_en(i);
      
      // Apply safety vetoes
      if (i == RPINLET && inlet_safety_veto) {
        target_state = ROFF;
      }
      // Note: Recovery pump veto is handled in loop_speedcontrol since it's a pump, not a relay pin here.
      
      if (target_state != getrelay(i)) {
        //Serial.println("In loop_pins: calling setrelay(" + String(i) + ", " + String(target_state) + ")");
        setrelay(i, target_state);
        rpins_changed = 1;
      }
    }

  }

  return;
}
