#include <Arduino.h>
#include "SPI.h"
#include "ADS1256.h"
#include "RFID.h"
#include "logger.h"

#define ZEROING_INTERVAL_SEC 10
#define ZEROING_INTERVAL_MS (ZEROING_INTERVAL_SEC * 1000)
#define MAX_DEPOSIT_TIMME_SEC (3 * 60)
#define MAX_DEPOSIT_TIMME_MS (MAX_DEPOSIT_TIMME_SEC * 1000)
#define SETTLING_TIMME_SEC 5
#define SETTLING_TIMME_MS (SETTLING_TIMME_SEC * 1000)
#define SPS 100
#define CAT_THRESH_G 1000
#define DEFAULT_BOX_WEIGHT_G 8000
#define NO_CHANGE_THRESH_G 10
#define POO_DETECTION_THRESH_G 2
#define MAX_DEPOSIT_LEN_S 300
#define MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS (SPS / 10) // transient should be less than 0.1 sec
#define RESETS_FOR_MEM_CLEAR 5 //num of resets within window to issue a clear cmd
#define RESET_WINDOW_MS 1000 // max time between consecutive resets such that they would count towards memory clear

RTC_DATA_ATTR uint8_t resetCount = 0;
RTC_DATA_ATTR unsigned long lastResetMs = 0;

typedef enum fsm_state
{
  idle,
  deposit,
  clean,
  post
};

volatile bool tagScanned = false;
volatile bool lidOpen = false;
static fsm_state state = idle;
static unsigned long elapsedTime_ms = 0;
static Logger logger;
bool clear_mem   = false;
bool clear_cred  = false;
bool clear_scale = false;

void setup()
{
  unsigned long now = millis();
  resetCount = (now - lastResetMs < RESET_WINDOW_MS) ? resetCount + 1 : 0;
  clear_mem = ( resetCount >= RESETS_FOR_MEM_CLEAR) ? true : false;
  lastResetMs = millis();
  SPI.begin(SCALE_CLK, SCALE_MISO, SCALE_MOSI);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  Serial.begin(115200);
  delay(10000);
  if (clear_mem)
  {
    resetCount = 0;
    Serial.println("Memory clear pattern detected, would you like to clear WiFi credentials? [Y/N]");
    while(true)
    {
      if (Serial.available())
      {
        if (Serial.readStringUntil('\n').startsWith("Y"))
        {
          clear_cred = true;
          Serial.println("WiFi credentials will be CLEARED!");
          break;
        }
        else if (Serial.readStringUntil('\n').startsWith("N"))
        {
          clear_cred = false;
          Serial.println("Will NOT clear WiFi credentials");
          break;
        }
      }
    }
    Serial.println("Would you like to clear Scale parameters? [Y/N]");
    while(true)
    {
      if (Serial.available())
      {
        if (Serial.readStringUntil('\n').startsWith("Y"))
        {
          clear_scale = true;
          Serial.println("Scale parameters will be CLEARED!");
          break;
        }
        else if (Serial.readStringUntil('\n').startsWith("N"))
        {
          clear_scale = false;
          Serial.println("Will NOT Scale pararmeters");
          break;
        }
      }
    }
  }
  logger.initLogger(clear_mem);
  initADS();
  Serial.println("ADS1256 Configured, starting sampling loop");
}

void loop()
{
  //FSM
  switch (state)
  {
    case idle:

      break;
    case deposit:
      break;
    case clean:
      break;
    case post:
      break;
  }
  //Idle - calibrate offset every 10 sec, if chip detected/large weight swing - switch to deposit. if lid off - switch to clean
  //Deposit - log all measurements (up to 5 minutes - 1MB), when weight drop: save new box weight, send measruments + tag_id to server, resume idle
  //Clean - wait for lid close, save new box weight and resume idler
  while(digitalRead(SCALE_DRDY))
  {
      delayMicroseconds(10);
  }
  // readAllCells();
  Serial.println((int)readAllCells());
}
