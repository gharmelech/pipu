#ifndef RFID_H
#define RFID_H

#include <Arduino.h>

#define RFID_RX_PIN 33
// #define RFID_RX_PIN 10
#define RFID_BAUD_RATE 9600
#define RFID_END_CHAR 03
#define RFID_TAG_LENGTH 15
#define RFID_TAG_HEX_LENGTH 13

#define TERRY 0x4F15432461871
#define KATANI 0x1AEF29B9201E3
#define JESSY 0x0AEF29B9201E3
#define TEST1 0x9F4390C3D3483
#define TEST2 0x605390C3D3483
#define TEST3 0x5FD67A9C42483

class rfidReader
{
private:
    void readRFID();
    const char *lastTagRead;
    bool newTag;
    HardwareSerial& _Serial;
public:
    rfidReader(HardwareSerial& serial) : _Serial(serial){};
    void begin();
    void restart();
    bool available();
    const char* getLastTagRead();
    ~rfidReader();
};

#endif