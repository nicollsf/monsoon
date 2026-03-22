// Fred Nicolls, March 2023


//HardwareSerial btSerial(2);  // pins GPIO16 (U2-Rx), GPIO17 (U2-Tx)
HardwareSerial &btSerial = Serial2;
#define RXD2 16
#define TXD2 17

#include "system.h"
#include "sensors.h"
#include "control.h"
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

void setup() 
{
  //wdt_disable();

  int serialm = 0;
  switch( serialm ) {
    case 0:
      Serial.begin(115200);
      btSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
      Serial.println("btSerial Txd is on pin: "+String(TXD2));
      Serial.println("btSerial Rxd is on pin: "+String(RXD2));
      break;
  }
  btLog("Main setup() called");

  setup_pins();
  setup_tempsens();
  setup_tempsens1();
  setup_flowsens();
  setup_psens();
  //setup_heatertriac();
  setup_speedcontrol();
  
  interrupts();

  //wdt_enable(WDTO_2S);     // enable watchdog
}

void loop() {
  //wdt_reset();  // still alive
  report_connblink();

  // Update sensor readings
  loop_tempsens();  // update temperatures (DS18B20)
  loop_tempsens1();  // update temperatures (NTC 10k)
  loop_flowsens();   // update flow sensor values
  loop_psens();  // update pressure sensor value

  // Process command input from bluetooth serial
  loop_btserialcmd();

  // Stuff to run for auto
  //loop_auto();

  // Actuation
  loop_heaters();
  loop_speedcontrol();
  loop_pins();
  if( rpins_changed ) rpins_lastreport = 0;  // force report

  // Timed report
  report_status();

  delay(10);
}


 