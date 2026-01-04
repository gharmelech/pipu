#include <Arduino.h>
#include "RFID.h"

bool *isScanned;
void RFID_ISR()
{
    *isScanned = true;
}
void initRFID(bool *tagScanned)
{
    Serial.printf("Starting RFID scanner");
    Serial1.begin(RFID_BAUD_RATE, SERIAL_8N1, RFID_RX_PIN);
    Serial1.onReceive(RFID_ISR);

}
char* readRFID()
{
  String readout = Serial1.readStringUntil(RFID_END_CHAR);
  char buf[RFID_TAG_LENGTH + 1];
  snprintf(buf, RFID_TAG_LENGTH + 1, "%d%llu", strtol(readout.substring(1, 5).c_str(), nullptr, 16), strtoull(readout.substring(5).c_str(), nullptr, 16));
  return buf;
}