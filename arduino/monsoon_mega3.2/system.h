#pragma once

#include <Arduino.h>


extern byte nwresets;  // number of warm restarts


// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

const int UNUSEDPINS[16] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52};  // shorted don't drive
const int RPINS0[8] = {23, 25, 27, 29, 31, 33, 35, 37};  // power relay board
const int RPINS1[8] = {39, 41, 43, 45, 47, 49, 51, 53};  // valve relay board
const int LSPINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};  // level sense inputs
const int LSPINSlv[8] = {HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};  // state for low indicator
const int FSPINS[2] = {2, 3};  // flow sensor inputs (on interrupt pins)
const int TSAPIN = A13;  // onewire digital
const int RSTSENS = 6;
const int SP_UND = A9, SP_CLK = A8;  // speed control
const int TRIAC_ZERO = 18, TRIAC_SCR = A10;  // heater control

const int RELAYOFF = HIGH, RELAYON = LOW;
extern int RPINS0_EN[8], RPINS1_EN[8];  // target values
extern int rpins_changed;
inline int rpinsen_reset(void) { for( int i=0; i<8; i++ ) RPINS0_EN[i] = RPINS1_EN[i] = 0; }

void setup_pins(void);
void loop_pins(void);
