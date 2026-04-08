#pragma once

#include <Arduino.h>

// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

// const int UNUSEDPINS[16] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52};  // shorted don't drive
const int RPINS[7] = { 18, 19, 23, 5, 13, 12, 2 };  // Relay output pins
const int RPINS_ROFF[7] = { HIGH, HIGH, HIGH, HIGH, LOW, LOW, LOW};  // Relay off states
//const int RPINS_ROFF[7] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW};  // Relay off states
const int LSPINS[2] = { 21, 22 };       // level sense inputs (Lsw low high)
const int LSPINSlv[2] = { LOW, HIGH };  // state for level low indicator
//const int LSCPINS[2] = { 25, 26 };       // level sense inputs (Lcap low high)
const int LSCPINS[2] = { 25, 34 };       // level sense inputs (Lcap low high).  For now placing one on blown pin 34 (probably not connected!)
const int LSCPINSlv[2] = { LOW, LOW };  // state for level low indicator
const int TSAPIN = 4;                   // temperature (onewire digital)
//const int TSA2PIN = 34;                 // temperature (NTC 10k analog)
//const int TSA2PIN = 26;                 // temperature (NTC 10k analog).  Want this on 34 but pin seems blown
const int TSA2PIN = 33;                 // temperature (NTC 10k analog).  Wanted this on 26 but pin seems blown (sinking current sometimes?)
const int FSPINS[2] = { 36, 39 };       // flow sensor inputs (on interrupt pins)
// const int RSTSENS = 6;
//const int SC_UND = 14, SC_CLK = 27;     // speed control (digipot)
//const int SC_EN = 14, SC_PWM = 27;      // speed control (hbridge)
const int SC_PWM0 = 14, SC_PWM1 = 27;      // speed control (hbridge)
// const int TRIAC_ZERO = 18, TRIAC_SCR = A10;  // heater control
//const int PSPIN = 25;                 // pressure sensor (analog in 0-4096)
const int PSPIN = 35;                   // pressure sensor (analog in 0-4096)
const int RLEVTXPIN = 15;               // level range TX
const int RLEVRXPIN = 32;               // level range RX

// Relay functionality
const int RPINLET = 0, RPDRAIN = 1, RPDELIVER = 2, RPHEATERA = 3;  // relay pin names
const int RPHEATER = 4, RPPUMP = 5, RPBURP = 6;  // relay pin names
extern int RPINS_EN[7];  // target values
const bool ROFF = 0, RON = 1;
extern int rpins_changed;
extern int htrs_forcedisable, htrs_enable;
inline void setrelay_en(int rpin, bool rval) { RPINS_EN[rpin] = rval; }
inline int getrelay_en(int rpin) { return RPINS_EN[rpin]; }
inline void setrelay(int rpin, bool rval) { digitalWrite(RPINS[rpin], rval==ROFF ? RPINS_ROFF[rpin] : !RPINS_ROFF[rpin]); }
inline int getrelay(int rpin) { return digitalRead(RPINS[rpin])==RPINS_ROFF[rpin] ? ROFF : RON; }
extern int rpins_reset_cnt;
inline void rpinsen_reset(void) { for( int i=0; i<7; i++ ) setrelay_en(i, ROFF); rpins_reset_cnt++; }

void setup_pins(void);
void loop_pins(void);