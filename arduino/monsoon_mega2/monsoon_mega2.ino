//Fred Nicolls, March 2018

HardwareSerial &btSerial = Serial3;


// ----------------------------------------------------------------------
//   Pin setup
// ----------------------------------------------------------------------

const int RPINS0[8] = {23, 25, 27, 29, 31, 33, 35, 37};  // power relay board
const int RPINS1[8] = {39, 41, 43, 45, 47, 49, 51, 53};  // valve relay board
const int LSPINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};  // level sense inputs
const int FSPINS[4] = {2, 3, 4, 5};  // flow sensor inputs (on interrupt pins)
const int TSAPIN = A13;  // onewire digital

const int LEVELHIGH = LOW, LEVELLOW = HIGH;
const int RELAYOFF = HIGH, RELAYON = LOW;

void setup_pins()
{
  // Digital outputs for relays (high is off)
  for( int i=0; i<8; i++ ) {
    pinMode(RPINS0[i], OUTPUT);  digitalWrite(RPINS0[i], RELAYOFF);  // power
    //rpins0_en[i] = 0;
    pinMode(RPINS1[i], OUTPUT);  digitalWrite(RPINS1[i], RELAYOFF);  // valves
  }
  report_rpins();

  // Digital inputs for flow sensors
  for( int i=0; i<4; i++ ) pinMode(FSPINS[i], INPUT_PULLUP);
  
  // Level sensors
  for( int i=0; i<8; i++ ) pinMode(LSPINS[i], INPUT_PULLUP);

  // Temperature setup on TSAPIN handled by onewire
}


// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

// Temperature sensor
#include <DallasTemperature.h>
#define ONE_WIRE_BUS TSAPIN
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

int temp_resolution = 12;
int temp_measureperiod = 5000;  // millis
int temp_convertdelay = 0;  // millis (calculated in setup from resolution)
int temp_idle;
unsigned long temp_lastconvertrequest = 0;
unsigned long temp_lastupdate = 0;
DeviceAddress temp0_DeviceAddress, temp1_DeviceAddress;
float temp0 = -1000.0, temp1 = -1000;
float temp_setpoint = 40.0;

void setup_tempsens(void) 
{ 
  Serial.println("Calling begin on DallasTemperature sensor"); 
  sensors.begin(); 

  int available = sensors.getDeviceCount();
  Serial.println("Sensors available: " + String(available));
  
  sensors.getAddress(temp0_DeviceAddress, 0);
  sensors.getAddress(temp1_DeviceAddress, 1);
  temp_convertdelay = 750 / (1 << (12 - temp_resolution));
  sensors.setResolution(temp0_DeviceAddress, temp_resolution);
  sensors.setResolution(temp1_DeviceAddress, temp_resolution);
 
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  temp_lastconvertrequest = millis();
  temp_idle = 0;  // conversion in progress
} 

void loop_tempsens(void) 
{ 
  // Start new conversion if required
  if( temp_idle==1 ) {
    if( millis()-temp_lastconvertrequest>=temp_measureperiod ) {
      //Serial.println("Requesting temperatures..."); 
      sensors.requestTemperatures();
      temp_lastconvertrequest = millis();
      temp_idle = 0;  // conversion in progress
    } 
    return;
  }

  // Handle ready temperature measurement
  temp0 = sensors.getTempCByIndex(0);  // first device on bus
  if( temp0<0 ) temp0 = 1000;
  temp1 = sensors.getTempCByIndex(1);
  if( temp1<0 ) temp1 = 1000;
  temp_lastupdate = millis();
  Serial.print("Temperatures: " + String(temp0) + "," + String(temp1));
  Serial.print(" (lastupdate=");  Serial.print(temp_lastupdate);  Serial.println(")");

  temp_idle = 1;  //  ready for next
} 

// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

volatile int fsvcnt0, fsvcnt1;
float flfreq0, flfreq1;
float flpm0, flpm1;
unsigned long flow_lastreset = 0;
const int flow_measureperiod = 1000;  // millis
void flowISR0(void) { fsvcnt0++; }
void flowISR1(void) { fsvcnt1++; }

void setup_flowsens(void) 
{ 
  attachInterrupt(digitalPinToInterrupt(FSPINS[0]), flowISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(FSPINS[1]), flowISR1, RISING);
  fsvcnt0 = 0;  fsvcnt1 = 0;
  flfreq0 = 0;  flfreq0 = 0;
  flpm0 = 0;  flpm1 = 0;
  flow_lastreset = millis();
}

void loop_flowsens(void) 
{ 
  if( millis()-flow_lastreset<flow_measureperiod ) return;
  
  unsigned int flowsens_duration = millis() - flow_lastreset;  
  flfreq0 = (float)fsvcnt0/flowsens_duration*1000.0;  // hertz
  flfreq1 = (float)fsvcnt1/flowsens_duration*1000.0;
  flpm0 = 10.0/82*flfreq0;  flpm1 = 10.0/82*flfreq1;  // 10lpm==82Hz?
  fsvcnt0 = 0;  fsvcnt1 = 0;
  flow_lastreset = millis();

  //btLog("FS counts (dur=" + String(flowsens_duration) + "): " + String(fsvcnt0) + " " + String(fsvcnt1));
  //btLog("FS hz=" + String(flfreq0) + "," + String(flfreq1) + " lpm?=" + String(flpm0) + "," + String(flpm1));
  btLog("FS lpm?=" + String(flpm0) + "," + String(flpm1));
}


// ----------------------------------------------------------------------
//   Simple auto functionality
// ----------------------------------------------------------------------

int rpins0_en[8];
int simpleauto_on = 0;
void setup_simpleauto(void) 
{
  simpleauto_on = 0;
  for( int i=0; i<8; i++ ) rpins0_en[i] = 0;
  report_simpleauto();
}

void loop_simpleauto(void)
{  
  if( !simpleauto_on ) return;  // only run if auto_on
  
  // For auto mode unsetting enable must turn output off
  for( int i=0; i<8; i++ ) {
    if( rpins0_en[i]==0 && digitalRead(RPINS0[i])==RELAYON ) {
      btLog("Relay " + String(i) + " OFF because enable=0");
      digitalWrite(RPINS0[i], RELAYOFF);  report_rpins();
    }
  }

  // Heaters and corresponding level sensor protection
  int rhtrs[4] = {0, 1, 2, 3};  // heater outputs
  int lshtrs[4] = {1, 1, 3, 3};  // level sensors protecting corresponding heaters
  float thtr[4];  // last measured heater temperatures
  thtr[0] = temp0;  thtr[1] = temp0;  thtr[2] = temp1;  thtr[2] = temp1;  // measured temps

  // Safe heaters
  for( int i=0; i<4; i++ ) {
    int rhtr = rhtrs[i], lshtr = lshtrs[i];
    if( digitalRead(RPINS0[rhtr])==RELAYON && digitalRead(LSPINS[lshtr])==LEVELLOW ) {
      digitalWrite(RPINS0[rhtr], RELAYOFF);  report_rpins();
      btLog("HTR " + String(i) + " OFF because LEV " + String(lshtr) + " low");
    }
  }

  // Auto heater functionality
  for( int i=0; i<4; i++ ) {
    int rhtr = rhtrs[i], lshtr = lshtrs[i];
    float temp = thtr[i];
    
    if( simpleauto_on && rpins0_en[rhtr] ) {
      if( temp<temp_setpoint && digitalRead(RPINS0[rhtr])==RELAYOFF && digitalRead(LSPINS[lshtr])==LEVELHIGH ) {
        digitalWrite(RPINS0[rhtr], RELAYON);  report_rpins();
        btLog("HTR " + String(i) + " on");
      }
      if( temp>temp_setpoint && digitalRead(RPINS0[rhtr])==RELAYON ) {
        digitalWrite(RPINS0[rhtr], RELAYOFF);  report_rpins();
        btLog("HTR " + String(i) + " off");
      }
    }
  }

  // Turn pumps on if required (no conditions for now)
  for( int i=4; i<6; i++ ) {
    if( rpins0_en[i] && digitalRead(RPINS0[i])==RELAYOFF ) {
      digitalWrite(RPINS0[i], RELAYON);  report_rpins();
    }
  }

  // Temporary shutdown
  if( rpins0_en[7] && digitalRead(LSPINS[1])==LEVELLOW ) {
    serialcmd('K');
  }
  
  report_simpleauto();
}


// ----------------------------------------------------------------------
//   Main auto functionality
// ----------------------------------------------------------------------


// Tap water from reservoir into working tank

bool keepworktankfull_enable = 0;
unsigned long keepworktankfull_lasttrig = 0;
const int keepworktankfull_period = 5000;
void loop_keepworktankfull(void)
{
  if( !keepworktankfull_enable ) return;
  if( millis()-keepworktankfull_lasttrig<keepworktankfull_period ) return;
  
  if( millis()-flow_lastreset<flow_measureperiod ) return;
  
  unsigned int flowsens_duration = millis() - flow_lastreset;  
  
  
}



// ----------------------------------------------------------------------
//   Serial command input
// ----------------------------------------------------------------------

String message; //string that stores the incoming message
void loop_btserialcmd(void) 
{
  while( btSerial.available() ) message += char(btSerial.read());  // construct message
  
  while( message!="" ) {  // if data is available
    char cmd = message[0];  message.remove(0,1);
    Serial.print("Processing comand: ");  Serial.println(cmd);  // echo
    serialcmd(cmd);
  }
}

void serialcmd(char cmd) 
{
  // Start of bluetooth disconnect message
  if( cmd=='+' ) {
    Serial.println("Disconnected");
    setup_pins();
  }
  
  // Kill
  if( cmd=='K' ) {
    btLog("KILL");  Serial.println("KILL");
    for( int i=0; i<8; i++ ) rpins0_en[i] = 0;
    simpleauto_on = 0; 
    setup_pins();  setup_simpleauto();
    report_rpins();  report_simpleauto();
  }

  // Auto mode
  if( cmd=='y' ) {
    simpleauto_on = !simpleauto_on;
    report_simpleauto();
  }

  if( cmd=='U' || cmd=='V') {
    if( cmd=='U') temp_setpoint += 0.5f;
    else temp_setpoint -= 0.5f;
    btLog("Temp setpoint now " + String(temp_setpoint));
    Serial.println("Temp setpoint now " + String(temp_setpoint));
  }
  
  // Power relays
  if( cmd>='a' && cmd<='h' ) {
    int rno = int(cmd) - int('a');  //Serial.println(rno);
    if( !simpleauto_on ) {
      int nval = !digitalRead(RPINS0[rno]);
      digitalWrite(RPINS0[rno], nval); 
      btLog("Power relay output " + String(rno) + " to " + String(nval));
      report_rpins();  
    } else {
      rpins0_en[rno] = !rpins0_en[rno];
      report_simpleauto();
    }
  }

  // Valve relays
  if( cmd>='A' && cmd<='H' ) {
    int rno = int(cmd) - int('A');  //Serial.println(rno);
    int nval = !digitalRead(RPINS1[rno]);
    digitalWrite(RPINS1[rno], nval); 
    btLog("Valve relay output " + String(rno) + " to " + String(nval));
    report_rpins();  
  }
}


// ----------------------------------------------------------------------
//   GUI reporting
// ----------------------------------------------------------------------

// Flashing light to indicate bluetooth connection
unsigned long connind_lastupdate = 0;
unsigned long connind_period = 750;
int connind_state = 0;  
void loop_connblink() 
{
  String rcmd = "*jR0G0B0*";
  String gcmd = "*jR0G255B0*";

  if( millis()-connind_lastupdate>=connind_period ) {
    connind_lastupdate = millis();
    connind_state = !connind_state;
    if( connind_state ) btSerial.println(rcmd);
    else btSerial.println(gcmd);
  } 
}

void btLog(String mess)
{
  btSerial.print("*T" + mess + + "\n" + "*");
}


// Report relay pin outputs
void report_rpins() {
  String mstr0 = "*i";
  for( int i=0; i<8; i++ ) {
    if( !digitalRead(RPINS0[i]) ) mstr0 += "1";
    else mstr0 += "0";
  }
  mstr0 += "*";

  String mstr1 = "*I";
  for( int i=0; i<8; i++ ) {
    if( !digitalRead(RPINS1[i]) ) mstr1 += "1";
    else mstr1 += "0";
  }
  mstr1 += "*";

  btSerial.println(mstr0);  //Serial.println(mstr0);
  btSerial.println(mstr1);  //Serial.println(mstr1);
}

// Report analog flow sensor values
unsigned long fsvals_lastreport = 0;
unsigned long fsvals_period = 500;
void report_fsvals() 
{
  if( millis()-fsvals_lastreport>=fsvals_period ) fsvals_lastreport = millis();
  else return;
  
  String mstr = "*N";
  mstr += String(flpm0) + "," + String(flpm1);
  mstr += "*";

  btSerial.println(mstr);  //Serial.println(mstr);
}

// Report analog temperature sensor values
unsigned long tsvals_lastreport = 0;
unsigned long tsvals_period = 750;
void report_tsvals() 
{
  if( millis()-tsvals_lastreport>=tsvals_period ) tsvals_lastreport = millis();
  else return;
  
  String mstr = "*M";
  mstr += String(temp0) + "," + String(temp1) + "*";

  btSerial.println(mstr);  //Serial.println(mstr);
}

// Report levels
void report_lsense(void)
{
  String mstr = "*Q";
  for( int i=0; i<4; i++ ) {
    if( digitalRead(LSPINS[i]) ) mstr += "1";
    else mstr += "0";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr0);
}

// Report auto mode enable status
void report_simpleauto(void)
{
  // Indicator for auto mode
  String oncmd = "*PR255G255B255*";
  String offcmd = "*PR0G0B0*";
  if( simpleauto_on ) {
    btSerial.println(oncmd);  Serial.println("Auto on");
  } else {
    btSerial.println(offcmd);  Serial.println("Auto off");
  }

  // Report enable flags
  String mstr = "*R";
  for( int i=0; i<8; i++ ) {
    if( rpins0_en[i] ) mstr += "1";
    else mstr += "0";
  }
  mstr += "*";
  btSerial.println(mstr);  //Serial.println(mstr0);
}


// ----------------------------------------------------------------------
//   Main setup and loop
// ----------------------------------------------------------------------

void setup() 
{
  btSerial.begin(9600);  Serial.begin(9600);

  setup_pins();
  setup_tempsens();
  setup_flowsens();
  setup_simpleauto();
  
  interrupts();
}

void loop() 
{
  loop_connblink();  // indicate connection status

  // Update sensor readings
  loop_tempsens();  report_tsvals();  // update temperatures
  loop_flowsens();  report_fsvals();  // update flow sensor values
  report_lsense();  // digital level sensor values
 
  // Process command input from bluetooth serial
  loop_btserialcmd();

  // Stuff to run if auto_on
  loop_simpleauto();
  
  delay(50);
}

