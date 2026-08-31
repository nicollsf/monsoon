// Fred Nicolls, March 2023

#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include <nvs_flash.h>

#include "gui.h"

LoggedSerial btSerial(Serial2);
#define RXD2 16
#define TXD2 17

#include "lwifi.h"
#include "ota.h"
#include "mqtt.h"

#include "system.h"
#include "sensors.h"
#include "control.h"
#include "auto.h"


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

#if ENABLE_WIFI || ENABLE_MQTT || ENABLE_OTA
void commsTask(void *pvParameters) {
  for (;;) {
#if ENABLE_WIFI
    loop_wifi();
#endif
#if ENABLE_OTA
    loop_ota();
#endif
#if ENABLE_MQTT
    loop_mqtt();
#endif
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks, run comms loop at ~100Hz
  }
}
#endif

void setup() 
{
#if ENABLE_WIFI || ENABLE_MQTT || ENABLE_OTA
  // Spawn the comms task on CPU Core 0, leaving CPU Core 1 dedicated to physical control loop safety
  xTaskCreatePinnedToCore(
    commsTask,
    "CommsTask",
    8192,        // Stack size in words (large enough for network client operations)
    NULL,        // Parameter
    1,           // Priority
    NULL,        // Task handle
    0            // Pinned to Core 0 (WiFi / network protocol stack core)
  );
#endif
  // Initialize NVS partition (required for fresh ESP32 modules)
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    err = nvs_flash_init();
  }
  
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

#if ENABLE_WIFI
  setup_wifi();
#endif
#if ENABLE_OTA
  setup_ota();
#endif
#if ENABLE_MQTT
  setup_mqtt();
#endif

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

  // Configure the task watchdog to 30 seconds
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 30000,      // 30 seconds timeout
      .idle_core_mask = 0,
      .trigger_panic = true     // Reboot on timeout
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);       // Subscribe current task (loopTask)
}

void loop() {
  esp_task_wdt_reset();  // still alive
  report_connblink();
  rpins_changed = 0;

  // Service comms - handled asynchronously by commsTask on CPU Core 0
  vTaskDelay(pdMS_TO_TICKS(1)); // Yield briefly to let other tasks share execution time
  
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


 