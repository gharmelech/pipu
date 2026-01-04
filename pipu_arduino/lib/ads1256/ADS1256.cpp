#include "ADS1256.h"
#include <Arduino.h>
#include "SPI.h"

static double calib_factors[4] = {0};
static int32_t offsets[4] = {0};

void sendCmd(uint8_t cmd)
{
    SPI.transfer(cmd);
}

void writeRegister(uint8_t reg,  uint8_t val)
{
    SPI.transfer(SCALE_CMD_WREG | reg);
    SPI.transfer(0x00);
    SPI.transfer(val);
}

uint8_t readRegister(uint8_t reg)
{
    uint8_t val = 0;
    SPI.transfer(SCALE_CMD_RREG | reg);
    SPI.transfer(0x00);
    delayMicroseconds(10);
    val = SPI.transfer(0xFF);
    Serial.printf("Register %X readback: %X\n", reg, val);
    return val;
}

int32_t readCurrentCell(bool toggleCS)
{
    uint8_t val = 0;
    int32_t sample = 0;
    if (toggleCS)
    {
        digitalWrite(SCALE_CS, LOW);
        delayMicroseconds(10);
    }
    sendCmd(SCALE_CMD_RDATA);
    delayMicroseconds(20);
    for (uint8_t i = 0; i < 3; i++)
    {
        val = SPI.transfer(0xFF);
        sample = (sample << 8) | val;
    }
    if (toggleCS)
    {
        digitalWrite(SCALE_CS, HIGH);
        delayMicroseconds(5);
    }
    sample = (sample & 0x800000) ? (0xFF000000 | sample) : sample;
    return sample;
}

float readAllCells(int32_t* ptr)
{
    int32_t cell_val = 0;
    float sample = 0;
    digitalWrite(SCALE_CS, LOW);
    delayMicroseconds(5);
    sendCmd(SCALE_CMD_SYNC);
    sendCmd(SCALE_CMD_WAKEUP);
    cell_val = readCurrentCell(false) - offsets[0];
    if (ptr != (int32_t*)nullptr)
    {
        *ptr = cell_val;
    }
    sample = sample + (double)cell_val / calib_factors[0];
    digitalWrite(SCALE_CS, HIGH);
    return sample;
}

void calibScale()
{
    uint8_t serialReadback = 0;
    int32_t reading = 0;
    int32_t calib_mass = 0;
    int32_t offset_i = 0;

    Serial.println("Starting scale calibration");
    
    Serial.println("Taring the scale, make sure it is level and clear of any objects, press \'r\' when ready to continue\n");
    while(true)
    {
        if (Serial.available())
        {
            Serial.readBytes(&serialReadback, 1);
            if (serialReadback == 'r')
            {
                Serial.printf("Taking reading for offsets\n");
                break;
            }
        }
    }
    for (int count = 0; count < 128; count++)
    {
        readAllCells(&reading);
        offset_i += reading;
    }
        offsets[0] = offset_i / 128;

    Serial.printf("Offsets measured: %d\n\n Now calibrating factor\n", offsets[0]);

    Serial.printf("Place mass at center, press \'r\' when ready\n");
    while(true)
    {
        if (Serial.available())
        {
            Serial.readBytes(&serialReadback, 1);
            if (serialReadback == 'r')
            {
                Serial.printf("Taking reading for calibration mass\n");
                break;
            }
        }
    }
    offset_i = 0;
    for (int i = 0; i < 255; i++)
    {
        readAllCells(&reading);
        offset_i += reading;
    }
    Serial.printf("Please enter weight in grams of calibration mass used\n");
    while(true)
    {
        if (Serial.available())
        {
            calib_mass = (double)Serial.parseInt();
            Serial.printf("Got: %d grams\n", calib_mass);
            break;
        }
    }
    calib_factors[0] = (double)reading / (double)calib_mass;
    Serial.printf("Calculated calibration factor: %.2f counts per gram\n", calib_factors[0]);

    delay(2000);
}

void initADS()
{
    digitalWrite (SCALE_RST, LOW);
    delayMicroseconds(10);
    digitalWrite (SCALE_RST, HIGH);
    while(digitalRead(SCALE_DRDY))
    {
        delayMicroseconds(10);
    }
    
    digitalWrite(SCALE_CS, LOW);
    delayMicroseconds(10);
    
    sendCmd(SCALE_CMD_SDATAC);
    writeRegister(SCALE_MUX_REG, SCALE_CELL0);
    writeRegister(SCALE_DRATE_REG, SCALE_2_5SPS); // TODO: change to 2000SPS
    writeRegister(SCALE_ADCON_REG, SCALE_GAIN64);
    delay(1000);
    readRegister(SCALE_MUX_REG);
    readRegister(SCALE_DRATE_REG);
    readRegister(SCALE_ADCON_REG);
    sendCmd(SCALE_CMD_SELFCALL);
    while(digitalRead(SCALE_DRDY))
    {
        delayMicroseconds(10);
    }
    sendCmd(SCALE_CMD_SYNC);
    sendCmd(SCALE_CMD_WAKEUP);

    digitalWrite(SCALE_CS, HIGH);
    calibScale();
}