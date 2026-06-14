#include <Arduino.h>
#include "RFID.h"

const char *terry = "Terry";
const char *katani = "Katani";
const char *jessry = "Jessy";
const char *test1 = "Test1";
const char *test2 = "Test2";
const char *test3 = "Test3";
const char unknown[] = "unknown";

void rfidReader::begin()
{
    _Serial.begin(RFID_BAUD_RATE, SERIAL_8N1, RFID_RX_PIN);
    _Serial.setTimeout(100);
    newTag = false;
    delay(1000);
    while (_Serial.available()){_Serial.read();} // flush rx buffer
    Serial.println("rfid reader ready!");
}

void rfidReader::restart()
{
    _Serial.end();
    delay(10);
    begin();
}

bool rfidReader::available()
{
    readRFID();
    return newTag;
}

const char* rfidReader::getLastTagRead()
{
    newTag = false;
    return lastTagRead;
}

void rfidReader::readRFID()
{
    if (_Serial.available())
    {
        Serial.println("reading tag");
        char readout[RFID_TAG_HEX_LENGTH + 20];
        // readout[RFID_TAG_HEX_LENGTH + 1] = '\0';
        if (_Serial.readBytesUntil(RFID_END_CHAR, readout, RFID_TAG_HEX_LENGTH + 10) >= RFID_TAG_HEX_LENGTH + 1)
        {
            readout[RFID_TAG_HEX_LENGTH + 1] = '\n';
            uint64_t tagNum = 0;
            tagNum = strtoull(readout + 1, nullptr, 16);
            Serial.printf("Read %s and parsed to %ull\n", readout, tagNum);
            for (int i = 0; i < RFID_TAG_HEX_LENGTH + 1; i++)
            {
                    Serial.print((uint8_t)readout[i], HEX);
                    Serial.print(" ");
            }
            Serial.println();
            switch (tagNum)
            {
            case TERRY:
                lastTagRead = terry;
                break;
            case KATANI:
                lastTagRead = katani;
                break;
            case JESSY:
                lastTagRead = jessry;
                break;
            case TEST1:
                lastTagRead = test1;
                break;
            case TEST2:
                lastTagRead = test2;
                break;
            case TEST3:
                lastTagRead = test3;
                break;
            default:
                lastTagRead = unknown;
                break;
            };
            newTag = true;
        }
        else
            Serial.println("invalid rfid scanner readout");
        while (_Serial.available()){_Serial.read();} // flush rx buffer
    }
}

rfidReader::~rfidReader(){}
