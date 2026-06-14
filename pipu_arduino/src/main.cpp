#include <Arduino.h>
#include <Preferences.h>
#include "SPI.h"
#include "ADS1256.h"
#include "RFID.h"
#include "logger.h"

// #define NO_SCALE
// #define SCALE_DEBUG

#define LID_GPIO 11
#define USER_BUTTON 0
#define SETTING_HOLD_THRESH_S 3 // seconds to hold user button in order to enter remote setting mode
#define REMOTE_SETTINGS_TIMEOUT_S 30 // max time to wait for remote setting reply
#define TICKS_PER_SEC 1000000
#define ZEROING_INTERVAL_SEC 10
#define REPORT_INTERVAL_SEC 60
#define MAX_DEPOSIT_TIMME_SEC (3 * 60)
#define MAX_DEPOSIT_TIMME_MS (MAX_DEPOSIT_TIMME_SEC * 1000)
#define SETTLING_TIMME_SEC 5
#define SPS 100
#define CAT_THRESH_G 2500
#define MIN_BOX_WEIGHT_G 3000
#define STEADY_STATE_THRESH_G 10
#define DROP_THRESH_G (-1)
#define SPIKE_THRESH_G 3
#define MAX_DEPOSIT_LEN_S 300
#define MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS (SPS / 5) // transient should be less than 0.2 sec
#define RESETS_FOR_MEM_CLEAR 5 //num of resets within window to issue a clear cmd
#define RESET_WINDOW_MS 5000 // max time between consecutive resets such that they would count towards memory clear

#define LED_TOGGLE digitalWrite(LED_BUILTIN, HIGH); delay(500); digitalWrite(LED_BUILTIN, LOW);

Preferences stateNVM;

enum fsm_state
{
  idle,
  deposit,
  clean
};

volatile SemaphoreHandle_t oneSecSemaphore;
volatile SemaphoreHandle_t lidSemaphore;
volatile uint32_t secCounter = 0;
volatile uint32_t depositTimerSec = 0;
static fsm_state state = idle;
static uint32_t loopCount = 0;
static uint32_t sampleCount = 0;
static uint32_t transientCoundown = 0;
static uint32_t lastReport = 0;
static uint32_t lastZero = 0;
static int32_t currentSample_g = 0;
static int32_t previousSample_g = 0;
static int32_t secSample_g[SETTLING_TIMME_SEC] = {0};
static int32_t windowMax_g = 0;
static int32_t windowMin_g = 0;
static int32_t windowAvg_g = 0;
static int32_t delta_g = 0;
static int32_t depositWeight = 0;
static int32_t catWeight = 0;
static bool isSec = false;
static bool steadyState = false;
static bool isNumberTwo = false;
Logger logger;
ADS1256 scale;
rfidReader rfid(Serial1);
static const char *catID;
static bool clear_mem   = false;
static bool clear_cred  = false;
static bool clear_scale = false;
static bool zeroing     = false;
hw_timer_t *oneSecTimer = NULL;

void remote_settings();

void IRAM_ATTR oneSecISR()
{
  secCounter++;
  xSemaphoreGiveFromISR(oneSecSemaphore, NULL);
  return;
}

void IRAM_ATTR lidISR()
{
  xSemaphoreGiveFromISR(lidSemaphore, NULL);
  return;
}

void setup()
{
  stateNVM.begin("resets_nvm", false);
  int32_t resetCount = stateNVM.getInt("rstCount");
  stateNVM.putInt("rstCount", resetCount + 1);
  delay(RESET_WINDOW_MS);
  clear_mem = false;
  if (resetCount >= RESETS_FOR_MEM_CLEAR)
  {
    clear_mem = true;
    stateNVM.putInt("rstCount", 0);
  }
  stateNVM.putInt("rstCount", 0);

  Serial.begin(115200);
  delay(5000);
  Serial.printf("Starting\n RST Cnt: %d\n", resetCount);
  
  if (clear_mem)
  {
    resetCount = 0;
    Serial.println("Memory clear pattern detected, would you like to clear WiFi credentials? [Y/N]");
    String res;
    while(true)
    {
      if (Serial.available())
      {

        res = Serial.readStringUntil('\n');
        if (res.startsWith("Y"))
        {
          clear_cred = true;
          Serial.println("WiFi credentials will be CLEARED!");
          break;
        }
        else if (res.startsWith("N"))
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
        res = Serial.readStringUntil('\n');
        if (res.startsWith("Y"))
        {
          clear_scale = true;
          Serial.println("Scale parameters will be CLEARED!");
          break;
        }
        else if (res.startsWith("N"))
        {
          clear_scale = false;
          Serial.println("Will NOT clear scale pararmeters");
          break;
        }
      }
    }
    res.clear();
  }
  logger.initLogger(clear_cred);
  rfid.begin();
#ifndef NO_SCALE
  SPI.begin(SCALE_CLK, SCALE_MISO, SCALE_MOSI);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  scale.begin(clear_scale);
  Serial.println("ADS1256 Configured, starting sampling loop");
#endif
  
  lidSemaphore = xSemaphoreCreateBinary();
  pinMode(USER_BUTTON, INPUT_PULLUP);
  pinMode(LID_GPIO, INPUT_PULLUP);
  attachInterrupt(LID_GPIO, lidISR, RISING);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  oneSecSemaphore = xSemaphoreCreateBinary();
  esp_timer_init();
  oneSecTimer = timerBegin(0, (APB_CLK_FREQ / TICKS_PER_SEC), true);
  timerAttachInterrupt(oneSecTimer, &oneSecISR, true);
  timerAlarmWrite(oneSecTimer, TICKS_PER_SEC, true);
  timerAlarmEnable(oneSecTimer);
}

void loop()
{
  if (!digitalRead(USER_BUTTON))
  {
    Serial.println("userbutton pressed");
    int32_t hold_cnt = 0;
    for (;hold_cnt < SETTING_HOLD_THRESH_S * 10; hold_cnt++) // check every 100ms
    {
      delay(100);
      if(digitalRead(USER_BUTTON))
        break;
    }
    if (hold_cnt == SETTING_HOLD_THRESH_S * 10)
    {
      Serial.println("Starting remote settings");
      // remote_settings();
      LED_TOGGLE
      scale.setManOffset(scale.getSample());
      while(!digitalRead(USER_BUTTON))
      {
        delay(100);
      }
    }
    //wait for box
    int32_t baseWeight = scale.getWeight();
    while(true)
    {
      delay(1000);
      if (scale.getWeight() - baseWeight > MIN_BOX_WEIGHT_G)
      {
        delay(500);
        scale.updateBoxWeight();
        logger.event_type("scale_tare");
        logger.event_sample(scale.getBoxWeight());
        logger.event_send();
        LED_TOGGLE
        delay(500);
        LED_TOGGLE
      }
    }
  }
  if (xSemaphoreTake(oneSecSemaphore, 0) == pdTRUE)
  {
    if (secCounter - lastZero >= ZEROING_INTERVAL_SEC)
    {
      zeroing = true;
      lastZero = secCounter;
    }
    if (secCounter - lastReport >= REPORT_INTERVAL_SEC)
    {
      Serial.printf("%d loop count in last %d seconds\n", loopCount, REPORT_INTERVAL_SEC);
#ifdef REPORT_LOOP
      logger.post_event(String("{\"Loop count\":" + loopCount + '}'));
#endif
      lastReport = secCounter;
      loopCount = 0;
    }
    isSec = true;
  }
  //FSM
#ifndef NO_SCALE
  previousSample_g = currentSample_g;
  currentSample_g = scale.getWeight();
#ifdef SCALE_DEBUG
  delay(1000); // reduce loop rate to 
  Serial.printf("current sample: %d grams\n", currentSample_g);
#else
  switch (state)
  {
    case idle:
      if (xSemaphoreTake(lidSemaphore, 0) == pdTRUE)
      {
        state = clean;
      }
      else if ((currentSample_g - scale.getBoxWeight() > CAT_THRESH_G) && isSec) // A cat is inside the box! align to sec counter
      {
        state = deposit;
        depositTimerSec = 0;
        sampleCount = 0;
        steadyState = false;
      }
      else if (zeroing)
      {
        scale.refreshOffset();
        zeroing = false;
      }
      break;
    case deposit:
      sampleCount++;
      logger.event_sample(currentSample_g);
      secSample_g[0] += currentSample_g;
      if (steadyState)
      {
        delta_g = currentSample_g - previousSample_g;
        if (!isNumberTwo)
        {
          if (delta_g < DROP_THRESH_G) // very small drop in weight
          {
            transientCoundown = MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS;
          }
          else if (transientCoundown > 0)
          {
            transientCoundown--;
            if (delta_g > SPIKE_THRESH_G) // weight spike
            {
              isNumberTwo = true;
            }
          }
        }
      }

      if (isSec) // full second
      {
        depositTimerSec++;
        secSample_g[0] = secSample_g[0] / SPS;
        if (((secSample_g[1] - secSample_g[0]) > CAT_THRESH_G )|| (depositTimerSec == MAX_DEPOSIT_LEN_S)) //cat left or timeout
        {
          if (rfid.available())
          {
            catID = rfid.getLastTagRead();
            logger.event_catID(String(catID));
          }

          if (isNumberTwo)
            logger.event_type("2");
          else
            logger.event_type("1");

          isNumberTwo = false;
          depositWeight = scale.getBoxWeight();
          scale.updateBoxWeight();
          depositWeight = scale.getBoxWeight() - depositWeight;
          logger.event_depositWeight(String(depositWeight));
          catWeight = windowAvg_g - depositWeight;
          logger.event_catWeight(String(catWeight));
          logger.event_send();
          #pragma unroll
          for (int i = SETTLING_TIMME_SEC; i >= 0; i--)
          {
            secSample_g[i] = 0;
          }
          state = idle;
        }
        else
        {
          if (!steadyState)
          {
            transientCoundown = 0;
            windowAvg_g = 0;
            windowMax_g = secSample_g[0];
            windowMin_g = secSample_g[0];
            for (int i = SETTLING_TIMME_SEC; i >= 0; i--)
            {
              windowMax_g = (secSample_g[i] > windowMax_g) ? secSample_g[i] : windowMax_g;
              windowMin_g = (secSample_g[i] < windowMin_g) ? secSample_g[i] : windowMin_g;
              windowAvg_g += secSample_g[i];
              if (i > 0)
              {
                secSample_g[i] = secSample_g[i - 1];
              }
            }
            windowAvg_g = windowAvg_g / SETTLING_TIMME_SEC;
            secSample_g[0] = 0;
            if (windowMax_g - windowMin_g <= STEADY_STATE_THRESH_G)
              steadyState = true;
          } 
        }
      }
      break;
    case clean:
      if (!digitalRead(LID_GPIO)) //reading low -> lid closed
      {
        delay(100); // debunce connection and verify
        if (!digitalRead(LID_GPIO)) //reading low -> lid closed
        {
          logger.event_sample(scale.getBoxWeight());
          scale.updateBoxWeight();
          logger.event_sample(scale.getBoxWeight());
          logger.event_type("Clean");
          logger.event_send();
          state = idle;
          rfid.restart();
        }
      }
      break;
  }
  #endif // #ifdef SCALE_DEBUG
#else// #ifndef NO_SCALE
    if (state == clean)
    {
      if (!digitalRead(LID_GPIO)) //reading low -> lid closed
      {
        delay(100); // debunce connection and verify
        if (!digitalRead(LID_GPIO)) //reading low -> lid closed
        {
          rfid.restart();
          state = idle;
        }
      }
    }
    else if (rfid.available())
    {
      logger.event_sample(1);
      catID = rfid.getLastTagRead();
      logger.event_catID(String(catID));
      logger.event_type("rfid_only");
      logger.event_send();
    }
#endif
  loopCount++;
  isSec = false;
}

inline bool send_remote_cmd(rcommand cmd, int32_t arg = 0)
{
  int32_t timeout = REMOTE_SETTINGS_TIMEOUT_S;
  while(timeout--)
  {
    if (logger.remote_setting(cmd, arg) != 0)
      break;
    delay(1000);
  }
  return ((timeout == 0) ? false : true);
}
void remote_settings()
{
  Serial.println("Entered remote settings mode");
  if (!send_remote_cmd(start))
    return;
  Serial.println("Offset cal");
  int32_t offset = scale.getSample();
  if (!send_remote_cmd(req_tare, offset))
    return;
  scale.setManOffset(offset);
  delay(100);
  if (!send_remote_cmd(ack_tare, scale.getSample() - offset))
    return;
  int32_t factor;
  Serial.println("Factor cal");
  do
  {
    factor = logger.remote_setting(req_calib, scale.getSample() - offset);
    delay(1000);
  } while (factor == 0);
  if (factor > 0)
  {
    scale.setManFactor(factor);
    logger.remote_setting(ack_calib, scale.getWeight());
  }
  Serial.println("Exiting remote settings");
}
