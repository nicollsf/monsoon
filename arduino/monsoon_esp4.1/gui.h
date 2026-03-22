#pragma once

#include "Arduino.h"

extern HardwareSerial &btSerial;

extern unsigned long rpins_lastreport;

void serialcmd(char cmd);
void loop_btserialcmd(void);
void report_connblink();
void btLog(String mess);
void report_status(void);
void report_tsvals(void);
void report_state(void);
