
// ----------------------------------------------------------------------
//   Speed control
// ----------------------------------------------------------------------
const int sc_resetperiod = 5000;  // millis
extern float sc_setperc;
void speedcontrol_set(int setperc, int resetflag);
void setup_speedcontrol(void);
void loop_speedcontrol(void);



// ----------------------------------------------------------------------
//   Heaters
// ----------------------------------------------------------------------

//extern float triac_Pp;
//void setup_heatertriac(void);

extern int htrs_disable;
extern int htrs_changed;
void loop_heaters(void);
