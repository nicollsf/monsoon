// Fred Nicolls Nov 2017

// Activate outputs
int ppump0_enable = 1;
int htr0_enable = 1;
int spump0_enable = 1;  

// Setup serial output
#include <SoftwareSerial.h>
SoftwareSerial btSerial(10, 11); // RX, TX
//HardwareSerial &logSerial = Serial;
SoftwareSerial &logSerial = btSerial;


// ----------------------------------------------------------------------
//   Pin definitions and setup
// ----------------------------------------------------------------------

// Interface
const int DBUTTON0 = 17;
const int DBUTTON1 = 16;  // broken too?
const int DBUTTON2 = 15;
const int DBUTTON3 = 14;  // seems broken
const int DLED0 = 18;
const int DLED1 = 19;
const int DLED2 = 21;
const int DLED3 = 20;  // assembled my controller box wrong...
const int ATEMPIN = A14;  // analog

// Relay output
const int DRELAY0 = 39;  // Primary pump
const int DRELAY1 = 41;  // Heater 0
const int DRELAY2 = 43;  // Heater 1
const int DRELAY3 = 45;  // DC scavenger pump
const int DRELAY4 = 47;
const int DRELAY5 = 49;
const int DRELAY6 = 51;
const int DRELAY7 = 53;

// Transistor output
const int DTRANS0 = 23;
const int DTRANS1 = 25;
const int DTRANS2 = 27;
const int DTRANS3 = 29;

// Sensors
const int ATEMPSENS = A8;  // analog
const int AFLOWSENS0 = A9;  // analog
const int AFLOWSENS1 = A10;  // analog
const int AFLOWSENS2 = A11;  // analog
const int AFLOWSENS3 = A12;  // analog
const int DURANGE_TRIG0 = 7;
const int DURANGE_ECHO0 = 6;
const int DURANGE_TRIG1 = 5;
const int DURANGE_ECHO1 = 4;
const int DLEVSENS0L = A1;
const int DLEVSENS0H = A2;
const int DLEVSENS1L = A0;
const int DLEVSENS1H = A3;
const int DLEVSENS2L = A4;
const int DLEVSENS2H = A5;
//const int DLEVSENS3B = A6;
//const int DLEVSENS3B = A7;


void setup_pins() {

  // Configure pushbuttons
  pinMode(DBUTTON0, INPUT_PULLUP);
  pinMode(DBUTTON1, INPUT_PULLUP);
  pinMode(DBUTTON2, INPUT_PULLUP);
  pinMode(DBUTTON3, INPUT_PULLUP);

  // Configure LED outputs
  pinMode(DLED0, OUTPUT);  digitalWrite(DLED0, LOW); 
  pinMode(DLED1, OUTPUT);  digitalWrite(DLED1, LOW);
  pinMode(DLED2, OUTPUT);  digitalWrite(DLED2, LOW);
  pinMode(DLED3, OUTPUT);  digitalWrite(DLED3, LOW);

  // Temperature setpoint input
  pinMode(ATEMPIN, INPUT);  // analog
  
  // Relay outputs.  High is off
  pinMode(DRELAY0, OUTPUT);  digitalWrite(DRELAY0, HIGH);
  pinMode(DRELAY1, OUTPUT);  digitalWrite(DRELAY1, HIGH);
  pinMode(DRELAY2, OUTPUT);  digitalWrite(DRELAY2, HIGH);
  pinMode(DRELAY3, OUTPUT);  digitalWrite(DRELAY3, HIGH);
  pinMode(DRELAY4, OUTPUT);  digitalWrite(DRELAY4, HIGH);
  pinMode(DRELAY5, OUTPUT);  digitalWrite(DRELAY5, HIGH);
  pinMode(DRELAY6, OUTPUT);  digitalWrite(DRELAY6, HIGH);
  pinMode(DRELAY7, OUTPUT);  digitalWrite(DRELAY7, HIGH);

  // Transistor outputs
  pinMode(DTRANS0, OUTPUT);  digitalWrite(DTRANS0, LOW); 
  pinMode(DTRANS1, OUTPUT);  digitalWrite(DTRANS1, LOW); 
  pinMode(DTRANS2, OUTPUT);  digitalWrite(DTRANS2, LOW); 
  pinMode(DTRANS3, OUTPUT);  digitalWrite(DTRANS3, LOW); 

  // Temperature analog input (but currently using onewire digital)
  //pinMode(ATEMPSENS, INPUT);  // analog
  
  // Flow sensors (analog)
  pinMode(AFLOWSENS0, INPUT);
  pinMode(AFLOWSENS1, INPUT);
  pinMode(AFLOWSENS2, INPUT);
  pinMode(AFLOWSENS3, INPUT);

  // Ultrasound ranging
  pinMode(DURANGE_TRIG0, OUTPUT);  digitalWrite(DURANGE_TRIG0, LOW); 
  pinMode(DURANGE_ECHO0, INPUT);
  pinMode(DURANGE_TRIG1, OUTPUT);  digitalWrite(DURANGE_TRIG1, LOW); 
  pinMode(DURANGE_ECHO1, INPUT);

  // Level sensing pins
  pinMode(DLEVSENS0L, INPUT);  pinMode(DLEVSENS0H, INPUT);
  pinMode(DLEVSENS1L, INPUT);  pinMode(DLEVSENS1H, INPUT);
  pinMode(DLEVSENS2L, INPUT);  pinMode(DLEVSENS2H, INPUT);
  //pinMode(DLEVSENS3L, INPUT);  pinMode(DLEVSENS3H, INPUT);
}

// ----------------------------------------------------------------------
//   Kill switch
// ----------------------------------------------------------------------

void loop_kill(void)
{
  if( !ppump0_enable && digitalRead(DRELAY0)==LOW ) {
    digitalWrite(DRELAY0, HIGH);  logSerial.println("Kill: primary pump off");
  }
  if( !htr0_enable && digitalRead(DRELAY1)==LOW ) {
    digitalWrite(DRELAY1, HIGH);  logSerial.println("Kill: heater off");
  }
  if( !spump0_enable && digitalRead(DRELAY3)==LOW ) {
    digitalWrite(DRELAY3, HIGH);  logSerial.println("Kill: scavenge pump off");
  }
}


// ----------------------------------------------------------------------
//   Temperature sensing
// ----------------------------------------------------------------------

// Temperature sensor
#include <DallasTemperature.h>
#define ONE_WIRE_BUS ATEMPSENS
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

int temp_resolution = 12;
int temp_measureperiod = 5000;  // millis
int temp_convertdelay = 0;  // millis (calculated in setup from resolution)
int temp_idle;
unsigned long temp_lastconvertrequest = 0;
unsigned long temp_lastupdate = 0;
DeviceAddress temp0_DeviceAddress;
float temp0 = -1000.0;
float temp0_setpoint = 50.0;

void setup_tempsens(void) 
{ 
  logSerial.println("Calling begin on DallasTemperature sensor"); 
  sensors.begin(); 

  sensors.getAddress(temp0_DeviceAddress, 0);
  temp_convertdelay = 750 / (1 << (12 - temp_resolution));
  sensors.setResolution(temp0_DeviceAddress, temp_resolution);
 
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
      //logSerial.println("Requesting temperatures..."); 
      sensors.requestTemperatures();
      temp_lastconvertrequest = millis();
      temp_idle = 0;  // conversion in progress
    } 
    return;
  }

  // Handle ready temperature measurement
  temp0 = sensors.getTempCByIndex(0);  // first device on bus
  if( temp0<0 ) temp0 = 1000;
  temp_lastupdate = millis();
  logSerial.print("Temperature 0 is: ");  logSerial.print(temp0);
  logSerial.print(" (lastupdate=");  logSerial.print(temp_lastupdate);  logSerial.println(")");

  temp_idle = 1;  //  ready for next
} 


// ----------------------------------------------------------------------
//   Level sensing
//   status:  -1 invalid, 0 low, 1 middle, 2 high (active low)
// ----------------------------------------------------------------------

unsigned long level_lastupdate = 0;

int level0_status = -1;  //int level1_status = -1;
unsigned long level0_ledlastchange = 0;  // for flashing
int l0l_last = -1, l0h_last = -1;
void loop_levelsens()
{
  int llow, lhigh;
  llow = digitalRead(DLEVSENS0L);  lhigh = digitalRead(DLEVSENS0H);

  // Interpret
  if( llow==1 ) {
    if( lhigh==1 ) level0_status = 0;  // low
    else level0_status = -1;  // invalid
  } else {
    if( lhigh==1 ) level0_status = 1;  // middle
    else level0_status = 2;  // high
  }

  level_lastupdate = millis();  

  if( llow!=l0l_last || lhigh!=l0h_last ) {
    logSerial.print("Level sensors (raw):  ");  logSerial.print(llow);   logSerial.print(" ");  logSerial.print(lhigh);   logSerial.println(" ");
  }
  
  l0l_last = llow;  l0h_last = lhigh;

  // Indicator led
  switch( level0_status ) {
    case -1:  // invalid
      if( level_lastupdate-level0_ledlastchange>50 ) {
        digitalWrite(DLED2, !digitalRead(DLED2));  level0_ledlastchange = level_lastupdate;
      }
      break;
    case 0:  // low
      digitalWrite(DLED2, LOW); 
      break;
    case 1:  // middle
      if( level_lastupdate-level0_ledlastchange>1000 ) {
        digitalWrite(DLED2, !digitalRead(DLED2));  level0_ledlastchange = level_lastupdate;
      }
      break;
    case 2:  // high
      digitalWrite(DLED2, HIGH); 
      break;
  }
  
}


// ----------------------------------------------------------------------
//   Flow sensing
// ----------------------------------------------------------------------

void loop_flowsens()
{
  logSerial.print("Flow sensors:  "); 
  logSerial.print(analogRead(AFLOWSENS0));   logSerial.print(" ");
  logSerial.print(analogRead(AFLOWSENS1));   logSerial.print(" ");
  logSerial.print(analogRead(AFLOWSENS2));   logSerial.print(" ");
  logSerial.print(analogRead(AFLOWSENS3));   logSerial.println(" ");
}


// ----------------------------------------------------------------------
//   Read outputs
// ----------------------------------------------------------------------

void loop_relayread()
{
  logSerial.print("Relay values:  "); 
  logSerial.print(digitalRead(DRELAY0));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY1));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY2));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY3));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY4));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY5));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY6));   logSerial.print(" ");
  logSerial.print(digitalRead(DRELAY7));   logSerial.println(" ");
}

void loop_transread()
{
  logSerial.print("Transister output values:  "); 
  logSerial.print(digitalRead(DTRANS0));   logSerial.print(" ");
  logSerial.print(digitalRead(DTRANS1));   logSerial.print(" ");
  logSerial.print(digitalRead(DTRANS2));   logSerial.print(" ");
  logSerial.print(digitalRead(DTRANS3));   logSerial.println(" ");
}


// ----------------------------------------------------------------------
//   System state
// ----------------------------------------------------------------------

enum State { STATE_INVALID, STATE_RESET, STATE_INITIALISING, STATE_READY, STATE_GO, STATE_STOPPING, STATE_STOPPED };
State state = STATE_INVALID;

// If active and required conditions met then on
int ppump0_active = 0;
int htr0_active = 0;
int spump0_active = 0;

void system_reset(void)
{
  ppump0_active = 0;  htr0_active = 0;  spump0_active = 0;
  digitalWrite(DRELAY0, HIGH);  digitalWrite(DRELAY1, HIGH);  digitalWrite(DRELAY3, HIGH);

  // Indicate reset
  digitalWrite(DLED0, LOW);  digitalWrite(DLED1, LOW);  digitalWrite(DLED2, LOW);  digitalWrite(DLED3, LOW);
  for( int i=0; i<6; i++ ) {
    digitalWrite(DLED0, !digitalRead(DLED0));
    digitalWrite(DLED1, !digitalRead(DLED1));
    digitalWrite(DLED2, !digitalRead(DLED2));
    digitalWrite(DLED3, !digitalRead(DLED3));
    delay(500);
  }
  digitalWrite(DLED0, HIGH);  digitalWrite(DLED1, LOW);  digitalWrite(DLED2, LOW);  digitalWrite(DLED3, LOW);

  state = STATE_RESET;
}

void system_setstate(State nstate=STATE_INVALID)
{
  // Call without arguments to advance current state
  if( nstate==STATE_INVALID ) {
    switch( state ) {
      case STATE_INVALID:  nstate = STATE_RESET;  break;
      case STATE_RESET:  nstate = STATE_INITIALISING;  break;
      case STATE_INITIALISING:  nstate = STATE_READY;  break;
      case STATE_READY:  nstate = STATE_GO;  break;
      case STATE_GO:  nstate = STATE_STOPPING;  break;
      case STATE_STOPPING:  nstate = STATE_STOPPED;  break;
    }
  }
  
  // Advance current
  switch( nstate ) {
    case STATE_RESET:
      system_reset();  
      break;
    case STATE_INITIALISING:
      ppump0_active = 0;  htr0_active = 1;  spump0_active = 1;
      break;
    case STATE_READY:
      ppump0_active = 0;  htr0_active = 1;  spump0_active = 1;
      break;
    case STATE_GO:
      ppump0_active = 1;  htr0_active = 1;  spump0_active = 1;
      break;
    case STATE_STOPPING:
      spump0_active = 1;  htr0_active = 0;  spump0_active = 1;
      spump0_active = 0;  // for testing
      break;
    case STATE_STOPPED:
      spump0_active = 0;  htr0_active = 0;  spump0_active = 0;
      break;
  }
  
  state = nstate;
}


unsigned long state_lastprint = 0;
void loop_stateupdate(void)
{
  // Get system into reset state
  if( state==STATE_INVALID ) {
    system_reset();
    return;
  }

  if( state==STATE_INITIALISING && temp0>temp0_setpoint && level0_status>=2 ) {
    system_setstate(STATE_READY);
  }

  if( state==STATE_READY || state==STATE_GO ) digitalWrite(DLED3, HIGH);
  else digitalWrite(DLED3, LOW);

  if( millis()-state_lastprint>=2500) {
    logSerial.print("Current state = ");
    switch( state) {
      case STATE_INVALID:  logSerial.println("STATE_INVALID");  break;
      case STATE_RESET:  logSerial.println("STATE_RESET");  break;
      case STATE_INITIALISING:  logSerial.println("STATE_INITIALISING");  break;
      case STATE_READY:  logSerial.println("STATE_READY");  break;
      case STATE_GO:  logSerial.println("STATE_GO");  break;
      case STATE_STOPPING:  logSerial.println("STATE_STOPPING");  break;
    }
    state_lastprint = millis();
  }
}


// ----------------------------------------------------------------------
//   Debounced buttons
// ----------------------------------------------------------------------

class diginput {
  int dpin;  // the number of the pin
  int state;  // current debounced state
  int lastvalue;  // the previous reading from the input pin
  unsigned long lasttime;  // the last time the input pin changed
  int debouncedelay;  // required stable time for reading

public:
  diginput(int pin, int initstate=HIGH) {
    dpin = pin;  pinMode(pin, INPUT);
    state = initstate;  lastvalue = initstate;  lasttime = 0;  debouncedelay = 100;
  }

  void update() 
  {
    int value = digitalRead(dpin);
    if( value!=lastvalue ) {
      lastvalue = value;
      lasttime = millis();
      return;
    }
    if( millis()-lasttime>=debouncedelay ) state = value;
    return;    
  }

  int pressed(int onrelease=FALSE)
  {
    int bstate = state;
    update();
    if( !onrelease && bstate==HIGH && state==LOW ) return(1);
    if( onrelease && bstate==LOW && state==HIGH ) return(1);
    return(0);
  }

};

diginput db0(DBUTTON0);  diginput db1(DBUTTON1);  diginput db2(DBUTTON2);  diginput db3(DBUTTON3);
void loop_buttons(void) 
{
  if( db0.pressed() ) system_setstate();  // advance state
  if( db1.pressed() ) {
    logSerial.println("Button 1 pressed");
    digitalWrite(DRELAY1, !digitalRead(DRELAY1));  // heater 0
    digitalWrite(DRELAY2, !digitalRead(DRELAY2));  // heater 1
  }
  if( db2.pressed() ) {
    logSerial.println("Button 2 pressed");
    digitalWrite(DRELAY3, !digitalRead(DRELAY3));  // pump
  }
  if( db3.pressed() ) system_setstate(STATE_RESET);
  //delay(10);
}


// ----------------------------------------------------------------------
//   Heater control
// ----------------------------------------------------------------------

void loop_heater()
{
  int htr0_prev = digitalRead(DRELAY1);  // current heater output

//  // Safety
//  if( (millis()-level_lastupdate>=5000) || (millis()-temp_lastupdate>=5000) ) {
//    digitalWrite(DRELAY1, HIGH);  // off
//    logSerial.println("htr0 off (level/temp not updating)");
//    return;
//  }
//
//  // Required off
//  if( htr0_prev==LOW ) {  // currently on
//    if( htr0_active==0 || level0_status<=0 || temp0>temp0_setpoint+0.5 ) {
//      digitalWrite(DRELAY1, HIGH);
//    }
//  }
//
//  // Desired on
//  if( htr0_active && htr0_prev==HIGH && level0_status>0 && temp0<temp0_setpoint-0.5 ) {
//    if( htr0_enable ) digitalWrite(DRELAY1, LOW);  
//  }

  // Output status
  digitalWrite(DLED1, !digitalRead(DRELAY1));
  if( htr0_prev!=digitalRead(DRELAY1) ) {
    if( digitalRead(DRELAY1) ) logSerial.println("htr0 is off");
    else logSerial.println("htr0 is on");
  }  
}


// ----------------------------------------------------------------------
//   Pump control
// ----------------------------------------------------------------------

void loop_pumps()
{
  int ppump0_prev = digitalRead(DRELAY0);  // current primary pump output
  int spump0_prev = digitalRead(DRELAY3);  // current scavenge pump output
  
  // Safety
  if( (millis()-level_lastupdate>=5000) ) {
    digitalWrite(DRELAY0, HIGH);  // off
    //digitalWrite(DRELAY3, HIGH);  // off
    logSerial.println("ppump0 and spump0 off (level not updating)");
    return;
  }

  // Primary require off
  if( ppump0_prev==LOW ) {  // currently on
    if( ppump0_active==0 ) {
      digitalWrite(DRELAY0, HIGH);
    }
  }

  // Primary desire on
  if( ppump0_active && ppump0_prev==HIGH ) {
    if( ppump0_enable ) digitalWrite(DRELAY0, LOW);  
  }
  
  // Scavenge require off
  if( spump0_prev==LOW ) {  // currently on
    if( spump0_active==0 || level0_status>=2 ) {
      //digitalWrite(DRELAY3, HIGH);
    }
  }

  // Scavenge desire on
  if( spump0_active && spump0_prev==HIGH && level0_status<2 ) {
    if( spump0_enable ) digitalWrite(DRELAY3, LOW);  
  }
  
}


// ----------------------------------------------------------------------
//   Loop tests
// ----------------------------------------------------------------------

void loop_relaytest() 
{
  //digitalWrite(DRELAY0, !digitalRead(DRELAY0));
  //digitalWrite(DRELAY1, !digitalRead(DRELAY1));
  //digitalWrite(DRELAY2, !digitalRead(DRELAY2));
  digitalWrite(DRELAY3, !digitalRead(DRELAY3));
  //digitalWrite(DRELAY4, !digitalRead(DRELAY4));
  //digitalWrite(DRELAY5, !digitalRead(DRELAY5));
  //digitalWrite(DRELAY6, !digitalRead(DRELAY6));
  //digitalWrite(DRELAY7, !digitalRead(DRELAY7));
  delay(3000);
}

void loop_transtest() {
  digitalWrite(DTRANS0, !digitalRead(DTRANS0));
  digitalWrite(DTRANS1, !digitalRead(DTRANS1));
  digitalWrite(DTRANS2, !digitalRead(DTRANS2));
  digitalWrite(DTRANS3, !digitalRead(DTRANS3));
}

void loop_levelsensetest(void)
{
  int reading0l = digitalRead(DLEVSENS0L);  int reading0h = digitalRead(DLEVSENS0H);
  int reading1l = digitalRead(DLEVSENS1L);  int reading1h = digitalRead(DLEVSENS1H);
  int reading2l = digitalRead(DLEVSENS2L);  int reading2h = digitalRead(DLEVSENS2H);
  //int reading3l = digitalRead(DLEVSENS3L);  int reading3h = digitalRead(DLEVSENS3H);
  //logSerial.print("Levelsensor readings: ");  
}

void loop_ledtest() 
{
  digitalWrite(DLED0, !digitalRead(DLED0));
  digitalWrite(DLED1, !digitalRead(DLED1));
  digitalWrite(DLED2, !digitalRead(DLED2));
  digitalWrite(DLED3, !digitalRead(DLED3));
  delay(500);
  logSerial.println("LEDs changed");
}

void loop_buttonstest() 
{
  logSerial.println("Running loop_buttonstest()");

  // Pushbuttons
  int reading;
  reading = digitalRead(DBUTTON0);  digitalWrite(DLED0, !reading);
  reading = digitalRead(DBUTTON1);  digitalWrite(DLED1, !reading);
  reading = digitalRead(DBUTTON2);  digitalWrite(DLED2, !reading);
  reading = digitalRead(DBUTTON3);  digitalWrite(DLED3, !reading);
  delay(10);        // delay in between reads for stability
}

//diginput db0(DBUTTON0);  diginput db1(DBUTTON1);  diginput db2(DBUTTON2);  diginput db3(DBUTTON3);
void loop_dbuttonstest()
{
  if( db0.pressed() ) digitalWrite(DLED0, !digitalRead(DLED0));
  if( db1.pressed() ) digitalWrite(DLED1, !digitalRead(DLED1));
  if( db2.pressed() ) digitalWrite(DLED2, !digitalRead(DLED2));
  if( db3.pressed() ) digitalWrite(DLED3, !digitalRead(DLED3));
  delay(10);
}

void loop_dbuttons1()
{
  if( db0.pressed() ) digitalWrite(DLED0, !digitalRead(DLED0));
  if( db1.pressed() ) digitalWrite(DLED1, !digitalRead(DLED1));
  //if( db2.pressed() ) digitalWrite(DLED2, !digitalRead(DLED2));
  if( db3.pressed() ) digitalWrite(DLED3, !digitalRead(DLED3));

  digitalWrite(DTRANS3, digitalRead(DLED0));
  //digitalWrite(DRELAY3, digitalRead(DLED1));
  //digitalWrite(DRELAY4, digitalRead(DLED3));
  
  delay(10);
}

void loop_dynamictest1(void) 
{
  // ACTIVE
  digitalWrite(DRELAY6, LOW);  // inlet
  digitalWrite(DRELAY7, LOW);  // outlet
  if(1) {
    digitalWrite(DRELAY0, HIGH);
    digitalWrite(DRELAY3, HIGH);
    //digitalWrite(DRELAY6, HIGH);  // inlet
    //digitalWrite(DRELAY7, HIGH);  // outlet
  } else {
    digitalWrite(DRELAY0, LOW);
    digitalWrite(DRELAY3, LOW);
    //digitalWrite(DRELAY6, LOW);  // inlet
    //digitalWrite(DRELAY7, LOW);  // outlet
  }
  delay(500);
} 

String message; //string that stores the incoming message
void loop_serialecho(void) 
{
  while( logSerial.available() ) message += char(logSerial.read());  // construct message

  if( !logSerial.available() ) {
    if( message!="" ) {  //if data is available
      logSerial.print("You said: ");  logSerial.println(message);  // echo
      message="";  //clear
    }
  }
}

// ----------------------------------------------------------------------
//   Main setup and loop
// ----------------------------------------------------------------------



void setup(void) 
{ 
  logSerial.begin(9600);  // serial comms

  setup_pins();
  setup_tempsens();
  digitalWrite(DRELAY7, LOW);  // valve
} 

void loop(void) 
{ 
  //loop_relaytest();  return;
  //loop_transtest();
  //loop_levelsensetest();
  //loop_ledtest();  return;
  //loop_buttonstest();  return;
  //loop_dbuttonstest();  return;
  //loop_dynamictest1();  return;
  
  loop_kill();  // check and disable if required
  loop_serialecho();

  // Instrumentation
  loop_levelsens();
  //loop_flowsens();
  loop_tempsens();

  // For information
  //loop_relayread();  loop_transread();

  loop_buttons();
  loop_stateupdate();

  // Control
  loop_heater();
  loop_pumps();
  
  delay(250);
} 


