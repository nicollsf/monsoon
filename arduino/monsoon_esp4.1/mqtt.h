#pragma once

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "ota.h"


// Update these with values suitable for your hardware/network.
//byte mac[]    = {  0x08, 0xB6, 0x1F, 0x3B, 0x67, 0x44 };
//IPAddress ip(172, 16, 0, 100);
//IPAddress server(34, 89, 101, 249);
const IPAddress server(10, 0, 0, 100);  // local RPi

void mqtt_log(String slog);
void mqtt_publish(String mqttstr);
void setup_mqtt();
void loop_mqtt();
