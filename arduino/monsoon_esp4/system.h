#pragma once

#include <Arduino.h>

// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

// const int UNUSEDPINS[16] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52};  // shorted don't drive
const int RPINS[7] = { 18, 19, 23, 5, 13, 12, 2 };  // Relay output pins
const int RPINS_ROFF[7] = { HIGH, HIGH, HIGH, HIGH, LOW, LOW, LOW};  // Relay off states
//const int LSPINS[2] = { 35, 34 };       // level sense inputs
const int LSPINS[2] = { 25, 26 };       // level sense inputs
const int LSPINSlv[2] = { LOW, HIGH };  // state for level low indicator
const int TSAPIN = 4;                   // temperature (onewire digital)
const int TSA2PIN = 34;                  // temperature (NTC 10k analog)
const int FSPINS[2] = { 36, 39 };       // flow sensor inputs (on interrupt pins)
// const int RSTSENS = 6;
const int SC_UND = 14, SC_CLK = 27;  // speed control (digipot)
const int SC_EN = 14, SC_PWM = 27;  // speed control (hbridge)
// const int TRIAC_ZERO = 18, TRIAC_SCR = A10;  // heater control
//const int PSPIN = 25;                   // pressure sensor (analog in 0-4096)
const int PSPIN = 35;                   // pressure sensor (analog in 0-4096)

//const int RELAYOFF = HIGH, RELAYON = LOW;
// extern int RPINS0_EN[8], RPINS1_EN[8];  // target values
extern int RPINS_EN[6];  // target values
extern int rpins_changed;
// inline int rpinsen_reset(void) { for( int i=0; i<8; i++ ) RPINS0_EN[i] = RPINS1_EN[i] = 0; }

void setup_pins(void);
void loop_pins(void);