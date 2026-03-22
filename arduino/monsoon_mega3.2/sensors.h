// Fred Nicolls Sept 2019
#pragma once


// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

const int temp_measureperiod = 5000;  // millis

extern float temp0, temp1;  // last measured temperatures
void setup_tempsens(void);
void loop_tempsens(void);


// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

const int flow_measureperiod = 1000;  // millis

extern int flow_lasts0, flow_lasts1;
extern unsigned long flow_lastch0, flow_lastch1;
extern float flow_lpm0, flow_lpm1;
void setup_flowsens(void);
void loop_flowsens(void);
