#include "ota.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include "ESP32OTAPull.h"

String ota_version = OTA_VERSION;


void ota_DisplayInfo()
{
	char exampleImageURL[256];
	snprintf(exampleImageURL, sizeof(exampleImageURL), "https://example.com/Basic-OTA-Example-%s-%s.bin", ARDUINO_BOARD, OTA_VERSION);

	Serial.printf("Basic-OTA-Example v%s\n", OTA_VERSION);
	Serial.printf("You need to post a JSON (text) file similar to this:\n");
	Serial.printf("{\n");
	Serial.printf("  \"Configurations\": [\n");
	Serial.printf("    {\n");
	Serial.printf("      \"Board\": \"%s\",\n", ARDUINO_BOARD);
	Serial.printf("      \"Device\": \"%s\",\n", WiFi.macAddress().c_str());
	Serial.printf("      \"Version\": %s,\n", OTA_VERSION);
	Serial.printf("      \"URL\": \"%s\"\n", exampleImageURL);
	Serial.printf("    }\n");
	Serial.printf("  ]\n");
	Serial.printf("}\n");
	Serial.printf("\n");
	Serial.printf("(Board, Device, Config, and Version are all *optional*.)\n");
	Serial.printf("\n");
	Serial.printf("Post the JSON at, e.g., %s\n", OTA_JSON_URL);
	Serial.printf("Post the compiled bin at, e.g., %s\n\n", exampleImageURL);
}

const char *ota_errtext(int code)
{
	switch(code)
	{
		case ESP32OTAPull::UPDATE_AVAILABLE:
			return "An update is available but wasn't installed";
		case ESP32OTAPull::NO_UPDATE_PROFILE_FOUND:
			return "No profile matches";
		case ESP32OTAPull::NO_UPDATE_AVAILABLE:
			return "Profile matched, but update not applicable";
		case ESP32OTAPull::UPDATE_OK:
			return "An update was done, but no reboot";
		case ESP32OTAPull::HTTP_FAILED:
			return "HTTP GET failure";
		case ESP32OTAPull::WRITE_ERROR:
			return "Write error";
		case ESP32OTAPull::JSON_PROBLEM:
			return "Invalid JSON";
		case ESP32OTAPull::OTA_UPDATE_FAIL:
			return "Update fail (no OTA partition?)";
		default:
			if (code > 0)
				return "Unexpected HTTP response code";
			break;
	}
	return "Unknown error";
}

void ota_callback(int offset, int totallength)
{
	Serial.printf("Updating %d of %d (%02d%%)...\n", offset, totallength, 100 * offset / totallength);

  // Publish current status
  char temp[512];
  sprintf(temp, "In ota_callback:  updating %d of %d (%02d%%)...\n", offset, totallength, 100 * offset / totallength);
  mqtt_log(temp);
}


void ota_update()
{
  ESP32OTAPull ota;
  ota.SetCallback(ota_callback);

  Serial.printf("Checking %s to see if an update is available...\n", OTA_JSON_URL);
  mqtt_log(String("In ota_update: ") + String("Checking for update at ") + String(OTA_JSON_URL));

	int ret = ota.CheckForOTAUpdate(OTA_JSON_URL, OTA_VERSION);
  //int ret = ota.CheckForOTAUpdate(OTA_JSON_URL, "0.0.0");  // force update
	Serial.println("In ota_update:  If the update succeeds the reboot should prevent us ever getting here");
	Serial.printf("In ota_update:  CheckForOTAUpdate returned %d (%s)\n", ret, ota_errtext(ret));
}


void setup_ota()
{
	//ota_DisplayInfo();
	Serial.printf("In setup_ota:  we are running version %s of the sketch, Board='%s', Device='%s'.\n", OTA_VERSION, ARDUINO_BOARD, WiFi.macAddress().c_str());

  // Publish current status
  char temp[512];
  sprintf(temp, "In setup_ota:  we are running version %s of the sketch, Board='%s', Device='%s'.\n", OTA_VERSION, ARDUINO_BOARD, WiFi.macAddress().c_str());
  mqtt_log(temp);

  //ota_update();
}


unsigned long ota_lastupdaterequest = 0;
const int ota_updatecheckperiod = 1000*60*5;  // millis
void loop_ota()
{
  if( millis()-ota_lastupdaterequest>=ota_updatecheckperiod ) {
    Serial.println("In loop_ota: calling scheduled ota_update");
    ota_update();
    ota_lastupdaterequest = millis();
  }
}

