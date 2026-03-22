#include "mqtt.h"

WiFiClient wifiClient;
PubSubClient client(wifiClient);

const unsigned long mqtt_reconnectinterval = 60000;
long mqtt_lastReconnectAttempt = 0;

void ota_update();  // in ota.h

void mqtt_log(String slog)
{
  Serial.println("In mqtt_log: calling publish on monsoon_outlog: " + slog);
  client.publish("monsoon_outlog", slog.c_str());
}

void mqtt_publish(String mqttstr)
{
  mqtt_log("In mqtt_publish: Publishing to monsoon_outTopic: " + mqttstr);
  client.publish("monsoon_outTopic", mqttstr.c_str());
}


void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  int i;

  // Serial.print("Message arrived [");
  // Serial.print(topic);
  // Serial.print("] ");
  // for( i=0; i<length; i++ ) {
  //   Serial.print((char)payload[i]);
  // }
  // Serial.println();

  char recvs[256];
  if( length>200 ) return;
  for( i=0; i<length; i++ ) recvs[i] = (char)payload[i];
  recvs[i] = '\0';
  String srecv = String(recvs);

  // // Echo
  // String echostr = "mqtt_callback [" + String(topic) + "]: " + String(srecv);
  // mqtt_log("In mqtt_callback: calling mqtt_publish(" + echostr + ")");
  // mqtt_publish(echostr);

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

}


boolean mqtt_reconnect() {
  Serial.println("In mqtt_reconnect: calling client.connect(\"monsoonClient\")");
  int ret = client.connect("monsoonClient");
  Serial.println("In mqtt_reconnect: client.connect() returned " + String(ret));
  if( ret ) {
    //Serial.println("In mqtt_reconnect: publishing hello world to monsoon_outTopic");
    mqtt_log("In mqtt_reconnect: publishing hello world to monsoon_outTopic");
    client.publish("monsoon_outTopic", "hello world");
    //mqtt_log("")
    //Serial.println("In mqtt_reconnect: subscribing to monsoon_inTopic");
    mqtt_log("In mqtt_reconnect: subscribing to monsoon_inTopic");
    client.subscribe("monsoon_inTopic");
  }
  return client.connected();
}

void setup_mqtt()
{
  client.setServer(server, 1883);
  client.setCallback(mqtt_callback);
  //Serial.println("In setup_mqtt: calling mqtt_reconnect");
  mqtt_log("In setup_mqtt: calling mqtt_reconnect");
  mqtt_reconnect();
}


void loop_mqtt()
{
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
    //Serial.println("In loop_mqtt: calling client.loop()");
    client.loop();
  }

}
