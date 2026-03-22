#include "gui.h"
#include "system.h"
#include "sensors.h"
#include "control.h"


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
    inchar = btSerial.read();

    if( inchar == -1 ) {
      btLog("btSerial.read() returned zero bytes");
      break;
    }

    // Greedy chomp AT message
    if( inchar == '+' ) {
      Serial.println("Chomping AT message: +");
      int bytesread = btSerial.readBytesUntil(endchar1, mess1, 30);
      mess1[bytesread] = endchar1;  mess1[bytesread + 1] = '\0';
      Serial.print(mess1);
      bytesread = btSerial.readBytesUntil(endchar2, mess2, 30);
      mess2[bytesread] = endchar2;  mess2[bytesread + 1] = '\0';
      Serial.print(mess2);  Serial.println("[Chomped]");
      break;
    }

    // Else execute command
    serialcmd(inchar);
  }
}

// Commands
// N kill; f-m power relays 1-8; F-M valve relays 1-8; U-V temp setpoint up/down
// S next next state; Y advance next state; P pause heaters
// A/a heater U/D; B/b speed U/D
void serialcmd(char cmd)
{
  Serial.print("Entered serialcmd with cmd=");  Serial.println(cmd);

  // Kill
  if( cmd == 'N' ) {
    btLog("KILL");
    setup_pins();
    setup_flowsens();
    setup_psens();
    setup_speedcontrol();
    interrupts();

    //auto_switchstate(STATE_OFF);  //auto_state = STATE_OFF;
    report_status();
  }

  // Software reset (restart)
  if( cmd == 'R') {
    btLog("RESET");
    ESP.restart();
  }

  // Relays (manual)
  if ( cmd >= 'f' && cmd <= 'l' ) {
    int rno = int(cmd) - int('f');  //Serial.println(rno);
    //rpinsv_get();
    //btLog("Before:  RPINS_EN[" + String(rno) + "] = " + String(RPINS_EN[rno]));
    RPINS_EN[rno] = !RPINS_EN[rno];
    //btLog("After:  RPINS_EN[" + String(rno) + "] = " + String(RPINS_EN[rno]));
  }

  // Speed control
  int setperc;
  if ( cmd == 'B' || cmd == 'b') {
    if( cmd=='B') {
      setperc = sc_setperc + 10;
      btLog("Calling speedcontrol_set(" + String(setperc) + ", false)");
      speedcontrol_set(setperc, false);
    } else {
      setperc = sc_setperc - 10;
      btLog("Calling speedcontrol_set(" + String(setperc) + ", false)");
      speedcontrol_set(setperc, false);
    }
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
  String mstr = "*i";
  for( int i=0; i<7; i++ ) {
    if( digitalRead(RPINS[i])==RPINS_ROFF[i] ) mstr += "0";
    else mstr += "1";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr0);

  mstr = "*k";
  if( RPINS_EN[4]==RPINS_ROFF[4]) mstr += "0";
  else mstr += "1";
  if( RPINS_EN[5]==RPINS_ROFF[5]) mstr += "0";
  else mstr += "1";
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr0);
  
  rpins_lastreport = millis();
}


// Report temperature
unsigned long tsvals_lastreport = 0;
unsigned long tsvals_period = 500;
unsigned long tsready_lastbeep = 0;
unsigned long tsready_beepperiod = 5000;
void report_tsvals()
{
  String mstr = "*M";
  mstr += String(temp0) + "," + String(temp1) + "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  mstr = "*t";
  mstr += String(temp_setpoint) + "*";
  btSerial.println(mstr);

//   if( auto_state==STATE_WARM && temp0>temp_setpoint && millis()-tsready_lastbeep>=tsready_beepperiod ) {
  if( temp0>temp_setpoint && millis()-tsready_lastbeep>=tsready_beepperiod ) {
    mstr = "*S*";
    btSerial.println(mstr);
    tsready_lastbeep = millis();
  }

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

  mstr = "*q";
  if( flow_lasts0 == 0 ) mstr += 0;
  else mstr += 1;
  if( flow_lasts1 == 0 ) mstr += 0;
  else mstr += 1;
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr);

  fsvals_lastreport = millis();
}


// Report levels
unsigned long lsvals_lastreport = 0;
unsigned long lsvals_period = 500;
void report_lsvals(void)
{
  String mstr = "*Q";
  for( int i=0; i<2; i++ ) {
    if( digitalRead(LSPINS[i])==LSPINSlv[i] ) mstr += "0";
    else mstr += "1";
  }
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


// Report all
void report_status(void)
{
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-tsvals_lastreport>=tsvals_period ) report_tsvals();
  if( millis()-fsvals_lastreport>=fsvals_period ) report_fsvals();
  if( millis()-lsvals_lastreport>=lsvals_period ) report_lsvals();
  if( millis()-psval_lastreport>=psval_period ) report_psvals();
  //if( millis()-state_lastreport>=state_period ) report_state();
  //if( millis()-progstat_lastreport>=progstat_period ) report_progstat();
  //if( millis()-debugstat_lastreport>=debugstat_period ) report_debugstat();
}