#include <EEPROM.h>  // persistence
extern const int eepromaddr0;

enum auto_states {
  STATE_NONE = 0, STATE_OFF, STATE_FILL, STATE_WARM, STATE_WASH, STATE_FLUSHE, STATE_FLUSHR, STATE_PAUSE, STATE_SHUT, STATE_CALIB
};

extern float temp_setpoint;
extern float temp_reqsetpoint;
extern auto_states auto_nextstate;
extern char *auto_statestrs[];
extern auto_states auto_state;
extern int auto_substate;
extern char *auto_substatestr;
extern unsigned long auto_statestime, auto_substatestime;

// Circuits
extern int auto_wtopupenable;//, auto_rtopupenable;
//extern int auto_wheaterenable, auto_rheaterenable;
//void loop_autortopup(void);
//void loop_autowtopup(void);
//void loop_autorheater(void);
//void loop_autowheater(void);

// Auto state loops
void loop_autooff(void);
void loop_autofill(void);
void loop_autowarm(void);
void loop_autowash(void);
void loop_autopause(void);
void loop_autoshut(void);
//void loop_calib(void);

// Primary auto loop
void loop_auto(void);

// State transitions
void auto_switchstate(int state);
void auto_switchsubstate(int substate);

extern float hstsampf[];
extern int htlnexti;
