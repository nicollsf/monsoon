#pragma once

// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

const int temp_measureperiod = 1000;  // millis

extern float temp0;  // last measured temperature (DS18B20)
extern float temp1;  // last measured temperature (NTC 10k)
extern float temp_setpoint;

// ----------------------------------------------------------------------
//   Temperature sensing (onewire DS18B20)
// ----------------------------------------------------------------------

void setup_tempsens(void);
void loop_tempsens(void);

// ----------------------------------------------------------------------
//   Temperature sensing (NTC 10k)
// ----------------------------------------------------------------------

void setup_tempsens1(void);
void loop_tempsens1(void);

// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

const int flow_measureperiod = 1000;  // millis

extern int flow_lasts0, flow_lasts1;
extern unsigned long flow_lastch0, flow_lastch1;
extern float flow_lpm0, flow_lpm1;
void setup_flowsens(void);
void loop_flowsens(void);


// ----------------------------------------------------------------------
//   Pressure sensing
// ----------------------------------------------------------------------

const int psens_measureperiod = 1000;  // millis

extern float psens0;  // last measured pressure (kPa)
void setup_psens(void);
void loop_psens(void);
