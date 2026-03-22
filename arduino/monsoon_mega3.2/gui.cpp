#include "gui.h"
#include "system.h"
#include "sensors.h"
#include "control.h"
#include "auto.h"


// ----------------------------------------------------------------------
//   Serial command input
// ----------------------------------------------------------------------

// Character-based code
void loop_btserialcmd(void)
{
  char inchar;
  char mess1[32], endchar1 = 'K';
  char mess2[32], endchar2 = 10;

  while ( btSerial.available() > 0 ) {
    inchar = btSerial.read();

    if ( inchar == -1 ) {
      btLog("btSerial.read() returned zero bytes");
      break;
    }

    // Greedy chomp AT message
    if ( inchar == '+') {
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
  if ( cmd == 'N' ) {
    btLog("KILL");
    setup_pins();
    auto_switchstate(STATE_OFF);  //auto_state = STATE_OFF;
    report_status();
  }

  // Temperature
  if ( cmd == 'U' || cmd == 'V') {
    if ( cmd == 'U') temp_reqsetpoint += 1.0f;
    else temp_reqsetpoint -= 1.0f;
    report_tsvals();
    btLog("Temp setpoint now " + String(temp_reqsetpoint));
    //Log("Temp setpoint now " + String(temp_reqsetpoint));
  }

  // Power relays (manual)
  if ( cmd >= 'f' && cmd <= 'm' ) {
    int rno = int(cmd) - int('f');  //Serial.println(rno);
    //rpinsv_get();
    btLog("Before:  RPINS0_EN[" + String(rno) + "] = " + String(RPINS0_EN[rno]));
    RPINS0_EN[rno] = !RPINS0_EN[rno];
    btLog("After:  RPINS0_EN[" + String(rno) + "] = " + String(RPINS0_EN[rno]));
    //int nval = !digitalRead(RPINS0[rno]);  digitalWrite(RPINS0[rno], nval);
    //btLog("Power relay output " + String(rno) + " to " + String(RPINS0_EN[rno]));
    //rpinsv_set();  report_rpins();
    
  }

  // Valve relays (manual)
  if ( cmd >= 'F' && cmd <= 'M' ) {
    int rno = int(cmd) - int('F');  //Serial.println(rno);
    //rpinsv_get();
    RPINS1_EN[rno] = !RPINS1_EN[rno];
    //int nval = !digitalRead(RPINS1[rno]);  digitalWrite(RPINS1[rno], nval);
    btLog("Output RPINS1_EN[" + String(rno) + "] to " + String(RPINS1_EN[rno]));
    //rpinsv_set();  report_rpins();
  }

  // Heater power
  if ( cmd == 'A' || cmd == 'a') {
    float nPpi;
    if ( cmd == 'A') {
      nPpi = triac_Pp + 5;  if ( nPpi>100 ) nPpi = 100;
    } else {
      nPpi = triac_Pp - 5;  if ( nPpi<0 ) nPpi = 0;
    }
    triac_Pp = nPpi;
    btLog("Heater power output " + String(triac_Pp));
  }

  // Speed control
//  if ( 0 && cmd == 'B' || cmd == 'b') {
//    float nSp = sc_Sp;
//    btLog("Speed control currently " + String(nSp));
//    if ( cmd == 'B') nSp += 5;
//    else nSp -= 5;
//    if ( nSp < 0 ) nSp = 0;
//    if ( nSp > 100 ) nSp = 100;
//    speedcontrol_set(nSp,1);
//    btLog("Speed control set to " + String(nSp));
//  }

  if ( cmd == 'B' || cmd == 'b') {
    if( cmd=='B') {
      btLog("Speed control 10 up");
      digitalWrite(SP_UND, HIGH);
    }
    else {
      btLog("Speed control 10 down");
      digitalWrite(SP_UND, LOW);
    }
    for ( int i = 0; i < 10; i++ ) {
      digitalWrite(SP_CLK, HIGH);  delayMicroseconds(3);
      digitalWrite(SP_CLK, LOW);  delayMicroseconds(3);
    }
  }

  // Advance next state specifier
  if ( cmd == 'S' ) {
    switch ( auto_nextstate ) {
      case STATE_OFF:  auto_nextstate = STATE_FILL;  break;
      case STATE_FILL:  auto_nextstate = STATE_WARM;  break;
      case STATE_WARM:  auto_nextstate = STATE_WASH;  break;
      case STATE_WASH:  auto_nextstate = STATE_PAUSE;  break;
      case STATE_PAUSE:  auto_nextstate = STATE_SHUT;  break;
      case STATE_SHUT:  auto_nextstate = STATE_CALIB;  break;
      case STATE_CALIB:  auto_nextstate = STATE_OFF;  break;
    }
    btLog("Incrementing auto_nextstate to " + String(auto_statestrs[auto_nextstate]));
    report_state();
  }

  // Enter next state
  if ( cmd == 'Y') {
    btLog("Advancing to state " + String(auto_statestrs[auto_nextstate]));
    auto_switchstate(auto_nextstate);  //auto_state = auto_nextstate;
  }

  // Toggle heater disable
  if ( cmd == 'P') {
    htrs_disable = !htrs_disable;
    btLog("htrs_disable = " + String(htrs_disable));
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

  if ( millis() - connind_lastupdate >= connind_period ) {
    connind_lastupdate = millis();
    connind_state = !connind_state;
    if ( connind_state ) btSerial.println(rcmd);
    else btSerial.println(gcmd);
  }
}

void btLog(String mess)
{
  static String lastmess;
  if ( mess == lastmess ) return;

  Serial.println("btLog: " + mess);
  btSerial.print("*L" + mess + "\n" + "*");
  lastmess = mess;
}

// Report relay pin outputs
unsigned long rpins_lastreport = 0;
unsigned long rpins_period = 500;
void report_rpins()
{
  String mstr0 = "*i";
  for ( int i = 0; i < 8; i++ ) {
    if ( !digitalRead(RPINS0[i]) ) mstr0 += "1";
    else mstr0 += "0";
  }
  mstr0 += "*";

  String mstr1 = "*I";
  for ( int i = 0; i < 8; i++ ) {
    if ( !digitalRead(RPINS1[i]) ) mstr1 += "1";
    else mstr1 += "0";
  }
  mstr1 += "*";

  if( 0 ) {
    String mstr2 = "";
    for ( int i = 0; i < 8; i++ ) {
      if ( RPINS0_EN[i]==1 ) mstr2 += "1";
      else mstr2 += "0";
    }
    Serial.println("RPINS0_EN = " + mstr2);
  }

  btSerial.println(mstr0);  //Serial.println(mstr0);
  btSerial.println(mstr1);  //Serial.println(mstr1);

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
  btSerial.println(mstr);

  mstr = "*t";
  mstr += String(temp_reqsetpoint) + "*";
  btSerial.println(mstr);

  if( auto_state==STATE_WARM && temp0>temp_setpoint && millis()-tsready_lastbeep>=tsready_beepperiod ) {
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
  if ( flow_lasts0 == 0 ) mstr += 0;
  else mstr += 1;
  if ( flow_lasts1 == 0 ) mstr += 0;
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
  for ( int i = 0; i < 4; i++ ) {
    if ( digitalRead(LSPINS[i]) == LSPINSlv[i] ) mstr += "0";
    else mstr += "1";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr0);

  lsvals_lastreport = millis();
}

// Report state
unsigned long state_lastreport = 0;
unsigned long state_period = 200;
void report_state(void)
{
  String mstr = "*s" + String(auto_statestrs[auto_state]) + ": " + auto_substatestr + "*";
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

// Report free memory
#include "MemoryFree.h"

unsigned long progstat_lastreport = 0;
unsigned long progstat_period = 20000;
void report_progstat(void)
{
  byte bauto_state = EEPROM.read(eepromaddr0);
  byte nwresets = EEPROM.read(eepromaddr0 + 1);
  unsigned long twashtime;
  EEPROM.get(eepromaddr0 + 2, twashtime);

  Serial.println("Program status:");
  Serial.print("  Free memory: ");  Serial.println(freeMemory());
  Serial.print("  Current state: ");  Serial.println(auto_statestrs[auto_state]);
  Serial.print("  Current saved state: ");  Serial.println(auto_statestrs[(auto_states)bauto_state]);
  Serial.print("  Warm reset count: ");  Serial.println((int)nwresets);
  Serial.print("  Total wash time (s): ");  Serial.println(twashtime / 1000.0f);
  if( auto_state==STATE_CALIB ) {
    Serial.print("  Total calibration time (s): ");  Serial.println((millis()-auto_substatestime) / 1000.0f);
  }
  Serial.println("Total wash time (s): " + String(twashtime / 1000.0f));

  String mstr = "*w" + String((int)nwresets) + "*";
  btSerial.println(mstr);  // warm reset count

  progstat_lastreport = millis();
}

// Debug status
unsigned long debugstat_lastreport = 0;
unsigned long debugstat_period = 30000;
void report_debugstat(void)
{
  //Serial.println("temp0,temp1=" + String(temp0) + "," + String(temp1) + " (temp_setpoint=" + String(temp_setpoint) + ")");

  Serial.println("Calibration state:");
  Serial.print("hstsampf = [ ");
  for( int i=0; i<htlnexti; i++ ) Serial.print(String(hstsampf[i]) + " ");
  Serial.println("]");

  debugstat_lastreport = millis();
}


// Report all
void report_status(void)
{
  if( millis()-rpins_lastreport>=rpins_period ) report_rpins();
  if( millis()-tsvals_lastreport>=tsvals_period ) report_tsvals();
  if( millis()-fsvals_lastreport>=fsvals_period ) report_fsvals();
  if( millis()-lsvals_lastreport>=lsvals_period ) report_lsvals();
  if( millis()-state_lastreport>=state_period ) report_state();
  if( millis()-progstat_lastreport>=progstat_period ) report_progstat();
  if( millis()-debugstat_lastreport>=debugstat_period ) report_debugstat();
}
