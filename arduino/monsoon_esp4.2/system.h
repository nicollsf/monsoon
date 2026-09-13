#pragma once

#include <Arduino.h>

#define ENABLE_WIFI 1  // Set to 0 to completely disable WiFi
#define ENABLE_MQTT 1  // Set to 0 to completely disable MQTT
#define ENABLE_OTA  1  // Set to 0 to completely disable OTA

// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

// const int UNUSEDPINS[16] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52};  // shorted don't drive
const int RPINS[8] = { 18, 19, 23, 4, 13, 21, 22, 5 };  // Relay output pins (Pin 5: Motorised Ball Valve)
const int RPINS_ROFF[8] = { HIGH, HIGH, HIGH, HIGH, LOW, LOW, LOW, HIGH };  // Relay off states
const int LSPINS[2] = { 36, 34 };       // level sense inputs (Lsw low low/high - input-only pins)
const int LSPINSlv[2] = { LOW, HIGH };  // state for level low indicator
const int LSCPINS[2] = { 12, 2 };       // level sense inputs (Lcap low high - retired to free pins)
const int LSCPINSlv[2] = { LOW, LOW };  // state for level low indicator
// const int TSAPIN = 5;                // temperature (retired, GPIO 5 now used for RPBALLVALVE)
const int TSA2PIN = 39;                 // temperature (NTC 15k analog - on VN input-only pin)
const int FSPINS[2] = { 25, 26 };       // flow sensor inputs (on interrupt pins D25 and D26)
const int SC_PWM0 = 14, SC_PWM1 = 27;   // speed control (hbridge)
const int PSPIN = 35;                   // pressure sensor (analog in 0-4096)
const int RLEVTXPIN = 15;               // level range TX
const int RLEVRXPIN = 32;               // level range RX

// Relay functionality
const int RPINLET = 0, RPDRAIN = 1, RPDELIVER = 2, RPHEATERA = 3;  // relay pin names
const int RPHEATER = 4, RPPUMP = 5, RPBURP = 6, RPBALLVALVE = 7;  // relay pin names
extern int RPINS_EN[8];  // target values
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