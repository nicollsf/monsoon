#include "mqtt.h"

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const unsigned long mqtt_reconnectinterval = 10000;
long mqtt_lastReconnectAttempt = 0;

#include "auto.h"

void ota_update();  // in ota.h

bool mqtt_clients_active = false;
unsigned long last_mqtt_client_ping = 0;

QueueHandle_t mqtt_queue = NULL;

void enqueue_mqtt_message(String topic, String payload) {
  extern unsigned long auto_statestime;
  if (topic == "monsoon_outTopic" && auto_state == STATE_OFF && (millis() - auto_statestime > 300000) && !mqtt_clients_active) {
    return; // Discard telemetry only when idle (STATE_OFF) for > 5 mins AND no active client is pinging
  }
  if (mqtt_queue == NULL) return;
  MqttMessage* msg = new MqttMessage{topic, payload};
  if (xQueueSend(mqtt_queue, &msg, 0) != pdTRUE) {
    delete msg; // Queue full, drop message to prevent memory leaks
  }
}

void mqtt_log(String slog)
{
  Serial.println("In mqtt_log: calling publish on monsoon_outlog: " + slog);
  enqueue_mqtt_message("monsoon_outlog", slog);
}

void mqtt_publish(String mqttstr)
{
  enqueue_mqtt_message("monsoon_outTopic", mqttstr);
}


void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Mark client as active on any incoming message/command
  last_mqtt_client_ping = millis();
  mqtt_clients_active = true;

  int i;
  char recvs[256];
  if( length>200 ) return;
  for( i=0; i<length; i++ ) recvs[i] = (char)payload[i];
  recvs[i] = '\0';
  String srecv = String(recvs);

  // Consume keepalive pings without running commands
  if (srecv == "p") {
    return;
  }

  if( srecv=="update" ) {
    Serial.println("In mqtt_callback:  calling OTA update");
    mqtt_log("In mqtt_callback:  calling OTA update");
    ota_update();
  }
  else if( srecv=="ping" ) {
    mqtt_publish("pong");
  }
  else if( srecv=="marco" ) {
    mqtt_publish("polo");
  }
  else if( length == 1 ) {
    extern void serialcmd(char cmd);
    serialcmd(srecv[0]);
  }

}


boolean mqtt_reconnect() {
  Serial.println("In mqtt_reconnect: calling client.connect(\"monsoonClient\")");
  int ret = client.connect("monsoonClient");
  Serial.println("In mqtt_reconnect: client.connect() returned " + String(ret));
  if( ret ) {
    mqtt_log("In mqtt_reconnect: publishing hello world to monsoon_outTopic");
    client.publish("monsoon_outTopic", "hello world");
    mqtt_log("In mqtt_reconnect: subscribing to monsoon_inTopic");
    client.subscribe("monsoon_inTopic");
  }
  return client.connected();
}

void setup_mqtt()
{
  mqtt_queue = xQueueCreate(30, sizeof(MqttMessage*));
  client.setServer(server, 1883);
  client.setCallback(mqtt_callback);
  client.setSocketTimeout(2); // Set socket timeout to 2s to prevent loop hanging during connect attempts
  mqtt_log("In setup_mqtt: calling mqtt_reconnect");
  mqtt_reconnect();
}


void loop_mqtt()
{
#if ENABLE_WIFI
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
#endif

  if( !client.connected() ) {
    //Serial.println("In loop_mqtt:  client not connected()");
    long now = millis();
    if( now-mqtt_lastReconnectAttempt>mqtt_reconnectinterval ) {
      mqtt_lastReconnectAttempt = now;
      //Serial.println("In loop_mqtt:  client status " + String(client.state()) + " so calling mqtt_reconnect()");
      mqtt_log("In loop_mqtt:  client status " + String(client.state()) + " so calling mqtt_reconnect()");
      if( mqtt_reconnect() ) mqtt_lastReconnectAttempt = 0;
      else {
        //Serial.println("In loop_mqtt:  mqtt_reconnect returned " + client.state());
        mqtt_log("In loop_mqtt:  mqtt_reconnect returned " + client.state());
      }
    }
  } else {
    // Check if clients have timed out (no activity for 25 seconds)
    if (mqtt_clients_active && (millis() - last_mqtt_client_ping > 25000)) {
      mqtt_clients_active = false;
      mqtt_log("No active clients. Suspending MQTT telemetry stream.");
    }

    //Serial.println("In loop_mqtt: calling client.loop()");
    client.loop();

    // Drain queued MQTT messages in the background (only if client is connected)
    if (mqtt_queue != NULL) {
      MqttMessage* msg;
      while (xQueueReceive(mqtt_queue, &msg, 0) == pdTRUE) {
        client.publish(msg->topic.c_str(), msg->payload.c_str());
        delete msg;
      }
    }
  }

}
