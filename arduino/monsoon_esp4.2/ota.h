#pragma once

#include <Arduino.h>
// #include <WiFiClient.h>
// #include <WiFi.h>
// #include "ESP32OTAPull.h"

#define OTA_JSON_URL   "http://10.0.0.9/ota/monsoon.json" // JSON filter file (see below)
#define OTA_VERSION    "1.0.82" // The current version of this program


void ota_update(void);
void mqtt_log(String slog);
void setup_ota(void);
void loop_ota(void);
void check_for_updates(void);
extern bool ota_pending;
extern bool ota_update_available;
extern String ota_available_version;


// Now create a text file called "Basic-OTA-Example.json" that describes the image you are going to post.  Example below.
// * The "Board" line, if provided, should match the ARDUINO_BOARD string that is predefined for the board you selected
// * If you include a "Device" line, the update will only match the single device that matches the provided MAC address.
//   Omit it if you want to update ALL devices.
// * The "Version" line should describe the posted image's version.  Update will only occur if it is different than VERSION.
// * Add a "Config" line to further filter if you like.  E.g. "Config": "32MB",
// * "URL" should point to the .bin file to be downloaded/installed

/*
	{
		"Configurations": [
			{
			"Board": "ESP32_DEV",
			"Device": "24:63:28:AD:FF:04",
			"Version": "1.0.1",
			"URL": "https://example.com/myimages/Basic-OTA-Example.bin"
			}
		]
	}
*/

/*
  Post the above file at <URL_JSON>.

  Compile your sketch with "Sketch/Export Compiled Binary", and post the resulting .bin file at the <URL> 
  you specified in the JSON file.
*/




