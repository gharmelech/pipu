#include "ADS1256.h"
#include <Arduino.h>
#include <Preferences.h>
#include "SPI.h"

static double calib_factor = 1;
static uint32_t offset = 0;

ADS1256::ADS1256(){}

void ADS1256::begin(bool clear)
{
    params.begin("scale_params", false);
    Serial.printf("Starting ADS1256, clear is set to %d\n", clear);
    if (clear)
    {
        params.clear();
    }

    pinMode(SCALE_RST, OUTPUT);
    pinMode(SCALE_CS, OUTPUT);
    pinMode(SCALE_DRDY, INPUT);

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
    writeRegister(SCALE_MUX_REG, CELL_PINS);
    writeRegister(SCALE_DRATE_REG, SCALE_100SPS); // TODO: change to 100SPS
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
    // Serial.println("Skipping calibration, don't forget to remove this!!!");
    calibrate();
}

ADS1256::~ADS1256()
{
}

void ADS1256::calibrate()
{
    if (params.isKey("calib_factor"))
    {
        calib_factor = params.getDouble("calib_factor");
        offset       = params.getInt("offset");
        box_weight = (params.isKey("box_weight")) ? params.getInt("box_weight") : 0;
        Serial.printf("Loaded calibration paramters: offset = %d, factor = %f\n", offset, calib_factor);
        return;
    }
    else
    {
        uint8_t serialReadback = 0;
        int32_t calib_mass = 0;
        int32_t reading = 0;

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
        for (int count = 0; count < 256; count++)
        {
            reading += getSample();
        }
        offset = reading / 256;
        params.putInt("offset", offset);
        Serial.printf("Offsets measured:%d counts.\n\n Now calculating calibration factor:\n", offset);

        Serial.printf("Place mass at center, press \'r\' when ready.\n");
        while(true)
        {
            if (Serial.available())
            {
                Serial.readBytes(&serialReadback, 1);
                if (serialReadback == 'r')
                {
                    Serial.printf("Taking reading for calibration mass...\n");
                    break;
                }
            }
        }
        reading = 0;
        for (int i = 0; i < 256; i++)
        {
            reading += getSample();
        }
        reading = reading / 256;
        Serial.printf("Please enter weight in grams of calibration mass used\n");
        while(true)
        {
            if (Serial.available())
            {
                calib_mass = Serial.parseInt();
                Serial.printf("Got: %u grams\n", calib_mass);
                break;
            }
        }
        calib_factor = (double)(reading - offset) / (double)calib_mass;
        Serial.printf("Calculated calibration factor: %.2f counts per gram\n", calib_factor);
        params.putDouble("calib_factor", calib_factor);
        
        reading = getWeight();
        Serial.printf("Valibration validation: Input - %d, readback - %d\n", calib_mass, reading);
        if ((reading >= calib_mass - MEASUREMENT_TOLERANCE_G) && (reading <= calib_mass + MEASUREMENT_TOLERANCE_G)) // meaurment within tolerance
        {
            Serial.printf("\t -> Calibration success! (tolerance = %dg)\n", MEASUREMENT_TOLERANCE_G);
            return;
        }
        else
        {
            Serial.printf("\t -> Calibration failed, clearing NVM params\n");
            params.clear();
            Serial.printf("Restart and try again!\n");
            while(1){}
        }    
    }
}

void ADS1256::refreshOffset()
{
    while(digitalRead(SCALE_DRDY)){} // wait for sample
    offset = getSample() - (int32_t)((double)box_weight * calib_factor);
    params.putInt("offset", offset); // update NVM
}

/**
 * @brief updates offset.
 *
 * @param setting new offset.
 */
void ADS1256::setManOffset(int32_t setting)
{
    offset = setting;
    params.putInt("offset", offset); // update NVM
}

/**
 * @brief updates calibration factor.
 *
 * @param setting calibration factor x 10,000.
 */
void ADS1256::setManFactor(int32_t setting)
{
    calib_factor = (double)setting / 10000.0;
    params.putDouble("calib_factor", calib_factor);
}

bool ADS1256::available()
{
    return (!digitalRead(SCALE_DRDY));
}

void ADS1256::updateBoxWeight(int32_t boxWeight)
{
    
    box_weight = boxWeight;
    params.putInt("box_weight", box_weight);
}

int32_t ADS1256::getSample()
{
    uint8_t val = 0;
    uint32_t sample = 0;

    digitalWrite(SCALE_CS, LOW);
    delayMicroseconds(10);
    sendCmd(SCALE_CMD_RDATA);
    delayMicroseconds(20);
    for (uint8_t i = 0; i < 3; i++)
    {
        val = SPI.transfer(0xFF);
        sample = (sample << 8) | val;
    }
    digitalWrite(SCALE_CS, HIGH);
    delayMicroseconds(5);
    sample = (sample & 0x800000) ? (0xFF000000 | sample) : sample;
    return (sample);
}

int32_t ADS1256::getWeight()
{
    int32_t sample = getSample();
    return (int)((double)(sample - offset) / calib_factor);
}

int32_t ADS1256::getBoxWeight()
{
    return (box_weight);
}

void ADS1256::sendCmd(uint8_t cmd)
{
    SPI.transfer(cmd);
}

void ADS1256::writeRegister(uint8_t reg,  uint8_t val)
{
    SPI.transfer(SCALE_CMD_WREG | reg);
    SPI.transfer(0x00);
    SPI.transfer(val);
}

uint8_t ADS1256::readRegister(uint8_t reg)
{
    uint8_t val = 0;
    SPI.transfer(SCALE_CMD_RREG | reg);
    SPI.transfer(0x00);
    delayMicroseconds(10);
    val = SPI.transfer(0xFF);
    Serial.printf("Register %X readback: %X\n", reg, val);
    return val;
}
