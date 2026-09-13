#pragma once


// ----------------------------------------------------------------------
//   Level sensing
// ----------------------------------------------------------------------

extern bool level_high0, level_high1;  // last reading bottom and top sensor
extern bool tank_full, tank_empty;
void update_tank_level_states(void);
extern bool level_chigh0, level_chigh1;  // last reading capacitive sensors
extern float level_hest;
extern float level_htank;

//extern int level_lastlt0, level_lastlt1;  // last time low
//extern int level_lastht0, level_lastht1;  // last time high
void setup_levelsens(void);
void loop_levelsens(void);


// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

const int temp_measureperiod = 1000;  // millis

// ----------------------------------------------------------------------
//   Temperature sensing (onewire DS18B20)
// ----------------------------------------------------------------------

extern float temp0;  // last measured temperature (DS18B20)
void setup_tempsens0(void);
void loop_tempsens0(void);

// ----------------------------------------------------------------------
//   Temperature sensing (NTC 10k)
// ----------------------------------------------------------------------

extern float temp1;  // last measured temperature (NTC 10k)
void setup_tempsens1(void);
void loop_tempsens1(void);

// ----------------------------------------------------------------------
// Temperature sensing (all)
// ----------------------------------------------------------------------

extern float temp;
void setup_tempsens(void);
void loop_tempsens(void);

// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

const int flow_measureperiod = 1000;  // millis

extern int flow_lasts0, flow_lasts1;
extern unsigned long flow_lastch0, flow_lastch1;
extern float flow_lpm0, flow_lpm1;
extern int flow_lastlt0, flow_lastlt1;
extern int flow_lastht0, flow_lastht1;
extern volatile unsigned long total_flow_pulses0, total_flow_pulses1;
extern float flow_rec_scale;
const int CALIBF_MAX_TABLE_PTS = 8;
extern int calibf_table_size;
extern float calibf_table_raw[CALIBF_MAX_TABLE_PTS];
extern float calibf_table_corr[CALIBF_MAX_TABLE_PTS];
float get_corrected_recovery_flow(float raw_rec_lpm);
void save_calibf_table(int n_pts, const float raw_pts[], const float corr_pts[]);
void load_calibf_table(void);
void flow_reset_total_pulses(void);
void setup_flowsens(void);
void loop_flowsens(void);


// ----------------------------------------------------------------------
//   Pressure sensing
// ----------------------------------------------------------------------

const int psens_measureperiod = 50;  // millis

extern float psens0;  // last measured pressure (kPa)
void setup_psens(void);
void loop_psens(void);


// ----------------------------------------------------------------------
//   Level (range) sensing
// ----------------------------------------------------------------------

extern int rlevdsens;
void setup_rlevsens(void);
void loop_rlevsens(void);