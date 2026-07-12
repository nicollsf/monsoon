#include "gui.h"
#include "system.h"
#include "sensors.h"
#include "control.h"
#include "auto.h"
#include "ota.h"


// ----------------------------------------------------------------------
//   Serial command input
// ----------------------------------------------------------------------

// Character-based code
void loop_btserialcmd(void)
{
  char inchar;
  char mess1[32], endchar1 = 'K';
  char mess2[32], endchar2 = 10;

  while( btSerial.available()>0 ) {
    //Serial.println("In loop_btserialcmd: btSerial.available=" + String(btSerial.available()));
    inchar = btSerial.read();
    //Serial.println("In loop_btserialcmd: btSerial.read() returned " + String(inchar));
    //Serial.print(String(inchar));

    if( inchar == -1 ) {
      btLog("btSerial.read() returned zero bytes");
      break;
    }

    // Else execute command
    //Serial.println("In loop_btserialcmd: calling serialcmd(" + String(inchar) + ")");
    serialcmd(inchar);
  }
}

// Commands
// N kill; f-m power relays 1-8; F-M valve relays 1-8; U-V temp setpoint up/down
// S next next state; Y advance next state; P pause heaters
// A/a heater U/D; B/b speed U/D
void serialcmd(char cmd)
{
  //Serial.print("Entered serialcmd with cmd=");  Serial.println(cmd);

  // Kill
  if( cmd == 'N' ) {
    btLog("KILL");
    setup_pins();
    rpinsen_reset();
    pumpsen_reset();
    setup_flowsens();
    setup_psens();
    setup_speedcontrol();
    interrupts();

    auto_switchstate(STATE_OFF);  //auto_state = STATE_OFF;
    report_status();
  }

  // Software reset (restart)
  if( cmd == 'R') {
    //btLog("RESET - called but temporarily disabled");
    ESP.restart();
  }

  // Change main auto mode
  if( cmd == 'a' ) {
    auto_double = !auto_double;
    btLog("Switching main mode auto_double=" + String(auto_double));
    rpinsen_reset();
    pumpsen_reset();
    auto_switchstate(STATE_OFF);
  }

  // Toggle disable heaters
  if( cmd == 'H' ) {
    htrs_forcedisable = !htrs_forcedisable;
    btLog("Toggle: htrs_forcedisable=" + String(htrs_forcedisable));
    report_state();
  }

  // Relays (manual)
  if ( cmd >= 'f' && cmd <= 'l' ) {
    int rno = int(cmd) - int('f');  //Serial.println(rno);
    setrelay_en(rno, !getrelay_en(rno));
  }

  // Speed control enable pumps
  if ( cmd == 'm' || cmd == 'n') {
    if( cmd=='m' ) {
      setpump_en(RPUMPR, !getpump_en(RPUMPR));
    } else {
      setpump_en(RPUMPD, !getpump_en(RPUMPD));
    }
  }

  // Speed control set speed
  int setperc;
  if ( cmd == 'B' || cmd == 'b') {
    if( cmd=='B') {
      btLog("Calling setpump_perc(RPUMPR, " + String(sc_setperc[RPUMPR] + 5) + ")");
      setpump_perc(RPUMPR, sc_setperc[RPUMPR] + 5);
    } else {
      btLog("Calling setpump_perc(RPUMPR, " + String(sc_setperc[RPUMPR] - 5) + ")");
      setpump_perc(RPUMPR, sc_setperc[RPUMPR] - 5);
    }
  }
  if ( cmd == 'C' || cmd == 'c') {
    if( cmd=='C') {
      btLog("Calling setpump_perc(RPUMPD, " + String(sc_setperc[RPUMPD] + 5) + ")");
      setpump_perc(RPUMPD, sc_setperc[RPUMPD] + 5);
    } else {
      btLog("Calling setpump_perc(RPUMPD, " + String(sc_setperc[RPUMPD] - 5) + ")");
      setpump_perc(RPUMPD, sc_setperc[RPUMPD] - 5);
    }
  }

  // Advance next state specifier
  if ( cmd == 'S' ) {
    switch ( auto_nextstate ) {
      case STATE_NONE:  auto_nextstate = STATE_OFF;  break;
      case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
      case STATE_FILL:  auto_nextstate = STATE_WARM;  break;
      case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
      case STATE_WASH:  auto_nextstate = STATE_RINSE;  break;
      case STATE_RINSE: auto_nextstate = STATE_PAUSE;  break;
      case STATE_PAUSE: auto_nextstate = STATE_SHUT;  break;
      case STATE_SHUT:  auto_nextstate = STATE_CALIBP;  break;
      case STATE_CALIBP:  auto_nextstate = STATE_CALIBT;  break;
      case STATE_CALIBT:  auto_nextstate = STATE_CALIBDUMP; break;
      case STATE_CALIBDUMP:  auto_nextstate = STATE_OFF;  break;
      case STATE_SETUP1: auto_nextstate = STATE_WARM1; break;
      case STATE_WARM1:  auto_nextstate = STATE_WASH1; break;
      case STATE_WASH1:  auto_nextstate = STATE_PAUSE; break;
    }
    Serial.println("Incrementing auto_nextstate to " + String(auto_statestrs[auto_nextstate]));
    report_state();
  }

  // Temperature setpoint
  if( cmd=='u' || cmd=='U' ) {
    if( cmd=='u' ) temp_setpoint -= 0.5;
    else temp_setpoint += 0.5;
    if( temp_setpoint>60.0 ) temp_setpoint = 40.0;
    save_temp_setpoint();
  }

  // Enter next state
  if ( cmd == 'Y') {
    btLog("Advancing to state " + String(auto_statestrs[auto_nextstate]));
    auto_switchstate(auto_nextstate);  //auto_state = auto_nextstate;
  }

  report_status();
}


// ----------------------------------------------------------------------
//   GUI reporting
// ----------------------------------------------------------------------

// j alive light; i power relay; I valve relay; q flow sense; Q level sense; L log window
// w restarts; M temp graphs; N flow graphs
// t temp setpoint; S beep; n next state; s current state and substate

// Flashing light to indicate bluetooth connection
unsigned long connind_lastupdate = 0;
unsigned long connind_period = 750;
int connind_state = 0;
void report_connblink()
{
  String rcmd = "*jR0G0B0*";
  String gcmd = "*jR255G255B0*";

  if( millis()-connind_lastupdate>=connind_period ) {
    connind_lastupdate = millis();
    connind_state = !connind_state;
    if( connind_state ) btSerial.println(rcmd);
    else btSerial.println(gcmd);
  }
}

void btLog(String mess)
{
  static String lastmess;
  if( mess==lastmess ) return;

  Serial.println("btLog: " + mess);
  btSerial.print("*L" + mess + "\n" + "*");
  lastmess = mess;
}

// Report relay pin outputs
unsigned long rpins_lastreport = 0;
unsigned long rpins_period = 500;
void report_rpins()
{
  String mstr;

  mstr = "*i";
  for( int i=0; i<7; i++ ) {
    if( getrelay_en(i)==ROFF ) mstr += "0";
    else mstr += "1";
  }
  for( int i=0; i<2; i++ ) {
    if( getpump_en(i)==ROFF ) mstr += "0";
    else mstr += "1";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  mstr = "*k";
  for( int i=0; i<7; i++ ) {
    if( getrelay(i)==ROFF ) mstr += "0";
    else mstr += "1";
  }
  for( int i=0; i<2; i++ ) {
    if( getpump_en(i)==ROFF ) mstr += "0";
    else mstr += "1";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);
  
  rpins_lastreport = millis();
}


// Report temperature
unsigned long tsvals_lastreport = 0;
unsigned long tsvals_period = 500;
unsigned long tsready_lastbeep = 0;
unsigned long tsready_beepperiod = 10000;
void report_tsvals()
{
  String mstr = "*M";
  //mstr += String(temp0) + "," + String(temp1) + "*";
  mstr += String(temp) + "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  mstr = "*t";
  mstr += String(temp_setpoint) + "*";
  btSerial.println(mstr);

 //if( auto_state==STATE_WARM && temp0>temp_setpoint && millis()-tsready_lastbeep>=tsready_beepperiod ) {
 //if( temp>temp_setpoint && millis()-tsready_lastbeep>=tsready_beepperiod ) {
 //  Serial.println("temp,temp_setpoint=" + String(temp) + "," + String(temp_setpoint));
 //  mstr = "*S*";
 //  btSerial.println(mstr);
 //  tsready_lastbeep = millis();
 //}

  tsvals_lastreport = millis();
}

// Report analog flow sensor values
unsigned long fsvals_lastreport = 0;
unsigned long fsvals_period = 500;
void report_fsvals()
{
  String mstr = "*N";
  mstr += String(flow_lpm0) + "," + String(flow_lpm1);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  // mstr = "*q";
  // if( flow_lasts0 == 0 ) mstr += 0;
  // else mstr += 1;
  // if( flow_lasts1 == 0 ) mstr += 0;
  // else mstr += 1;
  // mstr += "*";
  // btSerial.println(mstr);  //Serial.println(mstr);

  fsvals_lastreport = millis();
}


// Report levels
unsigned long lsvals_lastreport = 0;
unsigned long lsvals_period = 500;
void report_lsvals(void)
{
  String mstr = "*Q";

  // Mechanical sensors
  if( tank_empty ) mstr += "0";
  else mstr += "1";
  if( !tank_full ) mstr += "0"; // !tank_full means the upper sensor is low
  else mstr += "1";

  // Capacitive sensors
  if( !level_chigh0 ) mstr += "0";
  else mstr += "1";
  if( !level_chigh1 ) mstr += "0";
  else mstr += "1";

  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  // Percent full tank (estimated from flow)
  mstr = "*X";
  float level_hestp = level_hest/level_htank*100.0;
  mstr += String(level_hestp,0);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  // Percentage full tank (from ranging level sensor)
  mstr = "*Y";
  float level_hrp = (level_htank - (rlevdsens/10))/level_htank*100;
  mstr += String(level_hrp,0);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  lsvals_lastreport = millis();
}


// Report pressure
unsigned long psval_lastreport = 0;
unsigned long psval_period = 1000;
void report_psvals(void)
{
  String mstr = "*P";
  mstr += String(psens0,1);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  psval_lastreport = millis();
}


// Report speed control
unsigned long scval_lastreport = 0;
unsigned long scval_period = 500;
void report_scvals(void)
{
  String mstr = "*f";
  if( sc_setperc[RPUMPR]>99 ) mstr += "99";
  else mstr += String(sc_setperc[RPUMPR],0);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  mstr = "*F";
  if( sc_setperc[RPUMPD]>99 ) mstr += "99";
  else mstr += String(sc_setperc[RPUMPD],0);
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  scval_lastreport = millis();
}


// Report OTA version
// extern String ota_version;
unsigned long otaversion_lastreport = 0;
unsigned long otaversion_period = 1000;
void report_otaversion(void)
{
  String mstr = "*Z";
  mstr += String(OTA_VERSION);
  mstr += "*";
  btSerial.println(mstr);
  otaversion_lastreport = millis();
}


// Report state
unsigned long state_lastreport = 0;
unsigned long state_period = 200;
void report_state(void)
{
  String mstr;

  mstr = "*AR0G0B0*";
  if( auto_double ) mstr = "*AR255G255B255*";
  btSerial.println(mstr);

  mstr = "*HR255G0B0*";
  if( htrs_forcedisable || !htrs_enable ) mstr = "*HR0G0B0*";
  btSerial.println(mstr);

  if( auto_state==STATE_NONE ) mstr = "*sNONE*";
  else {
    const char *auto_substatestr = auto_substatestrs[auto_substate];
    mstr = "*s" + String(auto_statestrs[auto_state]) + ": " + auto_substatestr + "*";
  }
  btSerial.println(mstr);
    
  mstr = "*n" + String(auto_statestrs[auto_nextstate]) + "*";
  btSerial.println(mstr);

  //String mstr = "*a";
  //for( int i=0; i<4; i++ ) {
  //  if( digitalRead(LSPINS[i]) ) mstr += "1";
  //  else mstr += "0";
  //}
  //mstr += "*";
  //btSerial.println(mstr);  //Serial.println(mstr0);

  state_lastreport = millis();
}


// Report all
void report_status(void)
{
  if( millis()-otaversion_lastreport>=otaversion_period ) report_otaversion();
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-tsvals_lastreport>=tsvals_period ) report_tsvals();
  if( millis()-fsvals_lastreport>=fsvals_period ) report_fsvals();
  if( millis()-lsvals_lastreport>=lsvals_period ) report_lsvals();
  if( millis()-psval_lastreport>=psval_period ) report_psvals();
  if( millis()-scval_lastreport>=scval_period ) report_scvals();
  if( millis()-state_lastreport>=state_period ) report_state();
  //if( millis()-progstat_lastreport>=progstat_period ) report_progstat();
  //if( millis()-debugstat_lastreport>=debugstat_period ) report_debugstat();
}