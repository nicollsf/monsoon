#pragma once

#include <EEPROM.h>  // persistence
extern const int eepromaddr0;

extern int auto_double;  // two pump mode
enum auto_states {
  STATE_NONE = 0, STATE_OFF, STATE_FILL, STATE_SETUP1, STATE_WARM, STATE_WARM1, STATE_WASH, STATE_WASH1, STATE_RINSE, STATE_FLUSHE, STATE_FLUSHR, STATE_PAUSE, STATE_SHUT, STATE_CALIBP, STATE_CALIBT, STATE_CALIBDUMP, STATE_WTF
};

enum autowash_states {
  WASH_NONE = 0,
  WASH_SOFTSTART,
  WASH_CYCLE
};

extern float temp_setpoint;
extern float temp_reqsetpoint;
extern auto_states auto_nextstate;
extern const char *auto_statestrs[];
extern auto_states auto_state;
extern const char **auto_substatestrs;
extern int auto_substate;
//extern char *auto_substatestr;
extern unsigned long auto_statestime, auto_substatestime;

// Circuits
extern int auto_wtopupenable;//, auto_rtopupenable;
extern unsigned long auto_wtopup_interval;
extern int auto_woverflowstopenable;
//extern int auto_wheaterenable, auto_rheaterenable;
//void loop_autortopup(void);
//void loop_autowtopup(void);

// Auto state loops
void loop_autooff(void);
void loop_autofill(void);
void loop_autowarm(void);
void loop_autowash(void);
void loop_autorinse(void);
void loop_autopause(void);
void loop_autoshut(void);

// Primary auto loop
void setup_auto(void);
void loop_auto(void);

// State transitions
void auto_switchstate(int state, String reason = "");
void auto_switchsubstate(int substate);
void auto_advancestate(void);


extern float hstsampf[];
extern int htlnexti;
