// Fred Nicolls, March 2023

#include <Arduino.h>
#include <esp_system.h>

//HardwareSerial btSerial(2);  // pins GPIO16 (U2-Rx), GPIO17 (U2-Tx)
HardwareSerial &btSerial = Serial2;
#define RXD2 16
#define TXD2 17

#include "lwifi.h"
#include "ota.h"
#include "mqtt.h"

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

void setup() 
{
  //wdt_disable();
  htrs_enable = 1;

  int serialm = 0;
  switch( serialm ) {
    case 0:
      Serial.begin(115200);
      btSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
      Serial.println("btSerial Txd is on pin: " + String(TXD2));
      Serial.println("btSerial Rxd is on pin: " + String(RXD2));
      break;
  }

  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_TASK_WDT) {
    btLog("CRITICAL WARNING: System restarted due to Task Watchdog Timer (TWDT) reset!");
  } else if (reason == ESP_RST_INT_WDT) {
    btLog("CRITICAL WARNING: System restarted due to Interrupt Watchdog Timer (IWDT) reset!");
  } else if (reason == ESP_RST_WDT) {
    btLog("CRITICAL WARNING: System restarted due to Watchdog Timer (WDT) reset!");
  }

  btLog("Main setup() called");

  //setup_wifi();
  //setup_ota();
  //setup_mqtt();

  setup_pins();
  setup_levelsens();
  setup_rlevsens();
  setup_tempsens();
  setup_flowsens();
  setup_psens();
  //setup_heatertriac();
  setup_speedcontrol();
  setup_tempcontrol();

  Serial.println("setup: calling setup_auto");
  setup_auto();
  
  interrupts();

  enableLoopWDT();     // enable watchdog
}

void loop() {
  feedLoopWDT();  // still alive
  report_connblink();
  rpins_changed = 0;

  // Service comms
  //loop_wifi();
  //loop_ota();
  //loop_mqtt();
  
  // Update sensor readings
  loop_levelsens();
  loop_rlevsens();
  loop_tempsens();
  loop_flowsens();   // update flow sensor values
  loop_psens();  // update pressure sensor value

  // Process command input from bluetooth serial
  // Uncomment below for board with BT serial attached
  loop_btserialcmd();

  // Stuff to run for auto
  loop_auto();

  // Actuation
  loop_tempcontrol();
  loop_heaters();
  loop_pumps_and_valves(); // Centralized safety gatekeeper for pumps and valves
  loop_speedcontrol();
  loop_pins();
  if( rpins_changed ) rpins_lastreport = 0;  // force report

  // Timed report
  report_status();

  delay(10);
}


 