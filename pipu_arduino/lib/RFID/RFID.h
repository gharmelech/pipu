#ifndef RFID_H
#define RFID_H

#include <Arduino.h>

#define RFID_RX_PIN 10
#define RFID_BAUD_RATE 9600
#define RFID_END_CHAR 03
#define RFID_TAG_LENGTH 15

void initRFID(bool *tagScanned);
char* readRFID();

#endif