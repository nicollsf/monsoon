void setup_speedcontrol(void);
void loop_speedcontrol(void);

extern float triac_Pp;
void setup_heatertriac(void);

extern int htrs_disable;
extern int htrs_changed;
void loop_heaters(void);
