#pragma once

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "ota.h"


// Update these with values suitable for your hardware/network.
//byte mac[]    = {  0x08, 0xB6, 0x1F, 0x3B, 0x67, 0x44 };
//IPAddress ip(172, 16, 0, 100);
//IPAddress server(34, 89, 101, 249);
const IPAddress server(10, 0, 0, 7);  // local RPi

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct MqttMessage {
  String topic;
  String payload;
};

extern QueueHandle_t mqtt_queue;
extern bool mqtt_clients_active;
extern unsigned long last_mqtt_client_ping;

void mqtt_log(String slog);
void mqtt_publish(String mqttstr);
void setup_mqtt();
void loop_mqtt();
extern PubSubClient client;
