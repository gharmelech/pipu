#ifndef ADS_1256_H
#define ADS_1256_H

#include <cstdint>

// scale IO
#define SCALE_MISO 12
#define SCALE_MOSI 5
#define SCALE_CLK  16
#define SCALE_CS   37
#define SCALE_DRDY 35
#define SCALE_RST  39

// scale factors
#define SCALE_FACTOR0 41.453
#define SCALE_FACTOR1 41.582
#define SCALE_FACTOR2 41.435
#define SCALE_FACTOR3 42.935

// scale settings
#define SCALE_CELL0   0x10
#define SCALE_CELL1   0x23
#define SCALE_CELL2   0x45
#define SCALE_CELL3   0x67
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

void initADS();
int32_t readCurrentCell(bool toggleCS = true);
float readAllCells(int32_t* ptr = (int32_t*)nullptr);

#endif
