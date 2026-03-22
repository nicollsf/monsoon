//Fred Nicolls, August 2018


HardwareSerial &btSerial = Serial3;  // pins 14,15

#include "system.h"
#include "sensors.h"
#include "control.h"
#include "auto.h"
#include "gui.h"


// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
//
//      MAIN
//
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
//   Main setup and loop
// ----------------------------------------------------------------------

#include <avr/wdt.h>

void setup()
{
  wdt_disable();

  int serialm = 0;
  switch( serialm ) {
    case 0:
      btSerial.begin(115200);
      Serial.begin(115200);
      break;
    case 1:
      btSerial.begin(9600);
      Serial.begin(9600);
      break;
  }
  btLog("Main setup() called");
  
  // Hardware detect reset
  int rst = digitalRead(RSTSENS);
  btLog("Reset sense pin=" + String(rst));

  setup_pins();
  setup_tempsens();
  setup_flowsens();
  setup_heatertriac();
  setup_speedcontrol();

  interrupts();

  // Enable once and run to reset nonvolatiles
  if ( 0 ) {
    EEPROM.put(eepromaddr0, (byte)0);
    EEPROM.put(eepromaddr0 + 1, (byte)0);
    EEPROM.put(eepromaddr0 + 2, (unsigned long)0);
  }

  // Initial state
  auto_state = STATE_NONE;
  if ( rst == LOW ) {
    auto_switchstate(STATE_OFF);
    EEPROM.put(eepromaddr0 + 1, (byte)0);
  }
  else {  // warm reset detected
    byte bauto_state = EEPROM.read(eepromaddr0);
    auto_switchstate((auto_states)bauto_state);
    nwresets = EEPROM.read(eepromaddr0 + 1) + 1;
    EEPROM.put(eepromaddr0 + 1, nwresets); // count
    btLog("Warm reset number " + String(nwresets));
    btLog("Forced return to state " + String(auto_statestrs[(auto_states)bauto_state]));
  }

  triac_Pp = 80;  // include in saved state?  and control loop?

  wdt_enable(WDTO_2S);     // enable watchdog
}

void loop()
{
  wdt_reset();  // still alive
  report_connblink();  // indicate connection status

  // Update sensor readings
  loop_tempsens();  // update temperatures
  loop_flowsens();   // update flow sensor values

  // Process command input from bluetooth serial
  loop_btserialcmd();

  // Stuff to run for auto
  loop_auto();

  // Actuation
  loop_heaters();
  loop_speedcontrol();
  loop_pins();
  if( rpins_changed || htrs_changed ) rpins_lastreport = 0;  // force report

  // Timed report
  report_status();

  delay(10);
}
