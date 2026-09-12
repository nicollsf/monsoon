#pragma once

#include "Arduino.h"
#include "system.h"

class LoggedSerial : public Stream {
private:
  HardwareSerial &target;
  String tx_buffer;
public:
  LoggedSerial(HardwareSerial &ser) : target(ser), tx_buffer("") {}
  void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL, uint16_t rxfifo_size = 256) {
    target.begin(baud, config, rxPin, txPin, invert, timeout_ms, rxfifo_size);
  }
  size_t write(uint8_t c) override {
    if (c == '\n' || c == '\r') {
      if (tx_buffer.length() > 0) {
#if ENABLE_MQTT
        extern void mqtt_publish(String mqttstr);
        mqtt_publish(tx_buffer);
#endif
        tx_buffer = "";
      }
    } else {
      tx_buffer += (char)c;
    }
    return target.write(c);
  }
  int available() override { return target.available(); }
  int read() override {
    return target.read();
  }
  int peek() override { return target.peek(); }
  void flush() override { target.flush(); }
};

extern LoggedSerial btSerial;

extern unsigned long rpins_lastreport;

void serialcmd(char cmd);
void loop_btserialcmd(void);
void report_connblink();
void btLog(String mess);
void report_status(void);
void report_tsvals(void);
void report_state(void);
void report_rpins(void);
void report_valve_status(void);
