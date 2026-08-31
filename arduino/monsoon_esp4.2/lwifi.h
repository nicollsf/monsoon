#pragma once

#include "WiFi.h"

const int numssids = 5;
const String ssids[numssids] = {"Galaxy A04e", "abjaterHAHAHA", "abjaterHAHAHA_ext", "abjaterHAHAHA_ext2", "abjaterHAHAHA_ext3"};
const String ssidpasses[numssids] = {"1122334455", "1122334455", "1122334455", "1122334455", "1122334455"};

//const int numssids = 1;
//const String ssids[numssids] = {"Galaxy A04e"};
//const String ssidpasses[numssids] = {"1122334455"};

// Identify best network from above
int wifi_getbestssid(void);

//const unsigned long wifi_reconnectinterval = 20000;

// Handle wifi
void setup_wifi(void);
void loop_wifi(void);