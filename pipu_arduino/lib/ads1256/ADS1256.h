#ifndef ADS_1256_H
#define ADS_1256_H

#include <cstdint>
#include <Preferences.h>

// scale IO
#define SCALE_MISO 12
#define SCALE_MOSI 5
#define SCALE_CLK  16
#define SCALE_CS   37
#define SCALE_DRDY 35
#define SCALE_RST  39

// scale settings
#define CELL_PINS     0x10 // cell on pins 0 and 1
#define SCALE_2_5SPS  0x03
#define SCALE_10SPS   0x23
#define SCALE_100SPS  0x82
#define SCALE_1000SPS 0xA1
#define SCALE_2000SPS 0xB0
#define SCALE_GAIN64  0x06

//scale SPI commands
#define SCALE_CMD_WREG     0x50
#define SCALE_CMD_RREG     0x10
#define SCALE_CMD_RDATA    0x01
#define SCALE_CMD_SDATAC   0x0F
#define SCALE_CMD_WAKEUP   0x00
#define SCALE_CMD_SYNC     0xFC
#define SCALE_CMD_SELFCALL 0xF0
#define SCALE_CMD_RESET    0xFE

//scale registers
#define SCALE_MUX_REG   0x01
#define SCALE_ADCON_REG 0x02
#define SCALE_DRATE_REG 0x03
#define SCALE_OFC0_REG  0x05
#define SCALE_OFC1_REG  0x06
#define SCALE_OFC2_REG  0x07
#define SCALE_FSC0_REG  0x08
#define SCALE_FSC1_REG  0x09
#define SCALE_FSC2_REG  0x0A

//calibration validation
#define MEASUREMENT_TOLERANCE_G 5

class ADS1256
{
private:
    Preferences params;
    double calib_factor = 1.0;
    int32_t offset = 0;
    int32_t box_weight = 0;
    int32_t current_weight = 0;
    void calibrate();
    void sendCmd(uint8_t cmd);
    void writeRegister(uint8_t reg,  uint8_t val);
    uint8_t readRegister(uint8_t reg);
    
public:
    ADS1256();
    void begin(bool clear = false);
    void updateBoxWeight(int32_t boxWeight);
    void refreshOffset();
    void setManOffset(int32_t setting);
    void setManFactor(int32_t setting);
    bool available();
    int32_t getSample();
    int32_t getWeight();
    int32_t getBoxWeight();
    ~ADS1256();
};

void initADS();
int32_t getWeight();

#endif
