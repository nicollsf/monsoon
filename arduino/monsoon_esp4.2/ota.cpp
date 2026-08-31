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
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Skipping OTA update check: WiFi not connected");
    return;
  }

  ESP32OTAPull ota;
  ota.SetCallback(ota_callback);
  ota.AllowDowngrades(true);

  Serial.printf("Checking %s to see if an update is available...\n", OTA_JSON_URL);
  mqtt_log(String("In ota_update: ") + String("Checking for update at ") + String(OTA_JSON_URL));

	int ret = ota.CheckForOTAUpdate(OTA_JSON_URL, OTA_VERSION);
  //int ret = ota.CheckForOTAUpdate(OTA_JSON_URL, "0.0.0");  // force update
	Serial.println("In ota_update:  If the update succeeds the reboot should prevent us ever getting here");
	Serial.printf("In ota_update:  CheckForOTAUpdate returned %d (%s)\n", ret, ota_errtext(ret));
}


bool ota_update_available = false;
String ota_available_version = "";
bool ota_pending = false;

// Custom semver comparison helper to prevent alphabetical downgrade errors
bool is_newer_version(String current, String available) {
  int curr_parts[3] = {0, 0, 0};
  int avail_parts[3] = {0, 0, 0};
  
  // Parse current version
  int idx = 0;
  int start = 0;
  for (int i = 0; i < current.length() && idx < 3; i++) {
    if (current[i] == '.') {
      curr_parts[idx++] = current.substring(start, i).toInt();
      start = i + 1;
    }
  }
  if (idx < 3) curr_parts[idx] = current.substring(start).toInt();

  // Parse available version
  idx = 0;
  start = 0;
  for (int i = 0; i < available.length() && idx < 3; i++) {
    if (available[i] == '.') {
      avail_parts[idx++] = available.substring(start, i).toInt();
      start = i + 1;
    }
  }
  if (idx < 3) avail_parts[idx] = available.substring(start).toInt();

  // Compare semantic version parts numerically
  for (int i = 0; i < 3; i++) {
    if (avail_parts[i] > curr_parts[i]) return true;
    if (avail_parts[i] < curr_parts[i]) return false;
  }
  return false; // Equal version
}

void check_for_updates()
{
  if (WiFi.status() != WL_CONNECTED) return;

  ESP32OTAPull ota;
  ota.AllowDowngrades(true); // Allow JSON profile matching for verification

  int ret = ota.CheckForOTAUpdate(OTA_JSON_URL, OTA_VERSION, ESP32OTAPull::DONT_DO_UPDATE);
  if (ret == ESP32OTAPull::UPDATE_AVAILABLE) {
    String avail_ver = ota.GetVersion();
    if (is_newer_version(OTA_VERSION, avail_ver)) {
      ota_update_available = true;
      ota_available_version = avail_ver;
      Serial.printf("Background OTA Check: Update to v%s is available\n", ota_available_version.c_str());
      return;
    }
  }
  ota_update_available = false;
  ota_available_version = "";
  Serial.println("Background OTA Check: No newer version available");
}

void setup_ota()
{
	Serial.printf("In setup_ota:  we are running version %s of the sketch, Board='%s', Device='%s'.\n", OTA_VERSION, ARDUINO_BOARD, WiFi.macAddress().c_str());

  // Publish current status
  char temp[512];
  sprintf(temp, "In setup_ota:  we are running version %s of the sketch, Board='%s', Device='%s'.\n", OTA_VERSION, ARDUINO_BOARD, WiFi.macAddress().c_str());
  mqtt_log(temp);

  // We do NOT perform automatic update at setup anymore. Just verify connection on startup.
}

unsigned long ota_lastcheck = 0;
const unsigned long ota_check_interval = 1000 * 60 * 10; // Check every 10 minutes

void loop_ota()
{
  if (ota_pending) {
    ota_pending = false;
    ota_update();
  }

  // Periodic check for new firmware versions in the background
  if (WiFi.status() == WL_CONNECTED) {
    if (ota_lastcheck == 0 || millis() - ota_lastcheck >= ota_check_interval) {
      ota_lastcheck = millis();
      check_for_updates();
    }
  } else {
    ota_lastcheck = 0; // Force immediate check when WiFi reconnects
  }
}

