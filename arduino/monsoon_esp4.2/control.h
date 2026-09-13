#pragma once

#include <vector>
#include "system.h"

// ----------------------------------------------------------------------
//   Speed control
// ----------------------------------------------------------------------
const int sc_resetperiod = 5000;  // milli

// Pump functionality
const int RPUMPR = 0, RPUMPD = 1;  // Delivery and recovery
extern int RPUMP_EN[2];  // target values
extern float sc_setperc[2];  
extern int sc_pwm0[2];

inline void setpump_en(int rpump, bool rval) { RPUMP_EN[rpump] = rval; }
inline int getpump_en(int rpump) { return RPUMP_EN[rpump]; }
inline void setpump_perc(int rpump, float perc) 
{ 
  if( perc<0 ) perc = 0;
  
  float max_perc = (rpump == RPUMPD) ? 90.0f : 100.0f;
  if( perc>max_perc ) perc = max_perc;

  //Serial.println("Setting pump " + String(rpump) + " to " + String(perc) + "%");
  sc_setperc[rpump] = perc; 
}
inline void pumpsen_reset(void) { for( int i=0; i<2; i++ ) setpump_en(i, ROFF); }

// Pump calibration
struct pumpmodel {
  float offset; // PWM deadzone
  float a;      // Multiplier
  float b;      // Power (Curvature)
  float maxflow; // max LPM at 100% PWM
};
extern pumpmodel modelr, modeld;
pumpmodel pumpcalib_fit(const std::vector<float>& pwms, const std::vector<float>& flows);
void pumpcalib_savemodels(void);
void pumpcalib_loadmodels(void);
float getpwmFromlpm(float targetlpm, pumpmodel m);
void setpump_lpm(int rpump, float lpm);

// Main loop
void setup_speedcontrol(void);
void loop_speedcontrol(void);


// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

//extern float triac_Pp;
//void setup_heatertriac(void);

extern int htrs_enable;
extern int htrs_changed;
extern bool pump_safety_veto;
extern bool inlet_safety_veto;
void loop_heaters(void);
void loop_pumps_and_valves(void);


// ----------------------------------------------------------------------
//   Temperature control
// ----------------------------------------------------------------------
enum TCMODE { TCNONE = 0, TCOFF, TCON, TCHEATER, TCSPEED };
extern TCMODE temp_controlmode;
extern float temp_setpoint;
//float temp_reqsetpoint = temp_setpoint;
extern double tc_pidinput, tc_pidoutput;

void setup_tempcontrol(void);
void loop_tempcontrol(void);
void save_temp_setpoint(void);