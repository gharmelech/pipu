#include <Arduino.h>
#include <Preferences.h>
#include "SPI.h"
#include "ADS1256.h"
#include "RFID.h"
#include "logger.h"

// #define NO_SCALE
// #define SCALE_DEBUG
#define DEBUG_PRINTS 1
#define SEND_SNAPSHOT 1

#define LID_GPIO 11
#define USER_BUTTON 0
#define SETTING_HOLD_THRESH_S 3 // seconds to hold user button in order to enter remote setting mode
#define REMOTE_SETTINGS_TIMEOUT_S 30 // max time to wait for remote setting reply
#define TICKS_PER_SEC 1000000
#define SAMPLE_PRINT_INTERVAL_SEC 2
#define SNAPSHOT_INTERVAL_MIN 15
#define REPORT_INTERVAL_MIN 1
#define ZEROING_INTERVAL_SEC 30
#define MAX_ZEORING_DELTA_G 10
#define MAX_DEPOSIT_TIMME_SEC (3 * 60)
#define MAX_DEPOSIT_TIMME_MS (MAX_DEPOSIT_TIMME_SEC * 1000)
#define SPS 100
#define CAT_THRESH_G 2500
#define MIN_BOX_WEIGHT_G 2000
#define SETTLING_TIMME_SEC 3
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
static uint32_t snapshotLoopCount[SNAPSHOT_INTERVAL_MIN] = {0};
static uint32_t snapeshotTimer = SNAPSHOT_INTERVAL_MIN;
static uint32_t sampleCount = 0;
static uint32_t transientCoundown = 0;
static uint32_t lastSamplePrint = 0;
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
static bool zeroingInterval = false;
static bool oneSecInterval  = false;
static bool steadyState = false;
static bool isNumberTwo = false;
Logger logger;
ADS1256 scale;
rfidReader rfid(Serial1);
static const char *catID;
static bool clear_mem   = false;
static bool clear_cred  = false;
static bool clear_scale = false;
hw_timer_t *oneSecTimer = NULL;

void remote_settings();
void tare_scale();
void check_user_button();
void user_button_func();
void periodicals();
void idle_func();
void deposit_func();
void clean_func();
void rfid_only();
void scale_fsm();
void check_zero();

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
  if (!Serial)
  {
    Serial.end();
  }
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
  snapeshotTimer = SNAPSHOT_INTERVAL_MIN;
}

void loop()
{
  check_user_button();
  periodicals();
#ifndef NO_SCALE
  scale_fsm();
#else// #ifndef NO_SCALE
  rfid_only();
#endif
  loopCount++;
}

inline void scale_fsm()
{
  if (scale.available())
  {
    previousSample_g = currentSample_g;
    currentSample_g = scale.getWeight();
#ifdef SCALE_DEBUG
    delay(1000); // reduce loop rate to 1SPS
    Serial.printf("current sample: %d grams, %d counts\n", currentSample_g, scale.getSample());
#else
    switch (state)
    {
      case idle:
        idle_func();
        break;
      case deposit:
        deposit_func();
        break;
      case clean:
        clean_func();
        break;
    }
#endif // #ifdef SCALE_DEBUG
    oneSecInterval = false;
  }
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

inline void check_user_button()
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
      user_button_func();
  }
}

inline void user_button_func()
{
  detachInterrupt(LID_GPIO);
  // remote_settings();
  LED_TOGGLE
  delay(500);
  tare_scale();
  state = idle;
  attachInterrupt(LID_GPIO, lidISR, RISING);
}

inline void tare_scale()
{
  while(!digitalRead(USER_BUTTON)) //wait for button release
  {
    delay(100);
  }
  delay(1000);
  scale.setManOffset(scale.getSample());
  int32_t baseWeight = scale.getWeight();
  Serial.println("Scale tared");
  LED_TOGGLE // confirm offset saved
  delay(500);
  LED_TOGGLE
  //wait for box
  while(true)
  {
    delay(1000);
    Serial.println("Waiting for lid closure");
    digitalWrite(LED_BUILTIN, HIGH);
    // if ((scale.getWeight() - baseWeight > MIN_BOX_WEIGHT_G) && (!digitalRead(LID_GPIO)))
    if (!digitalRead(LID_GPIO))
    {
      delay(1000);
      Serial.println("Lid closed, updating box weight");
      scale.updateBoxWeight();
      Serial.println("Box weight updated, sending event");
      logger.event_type("scale_tare");
      logger.event_sample(baseWeight);
      logger.event_sample(scale.getBoxWeight());
      logger.event_send();
      LED_TOGGLE
      delay(500);
      LED_TOGGLE
      break;
      state = idle;
    }
  }
  Serial.println("Event sent, resuming normal operation");
}

inline void periodicals()
{
  if (xSemaphoreTake(oneSecSemaphore, 0) == pdTRUE)
    {
      if (secCounter - lastZero >= ZEROING_INTERVAL_SEC)
      {
        zeroingInterval = true;
        lastZero = secCounter;
      }
#if DEBUG_PRINTS
      if (secCounter - lastSamplePrint >= SAMPLE_PRINT_INTERVAL_SEC)
      {
        Serial.printf("%d\n", previousSample_g);
        lastSamplePrint = secCounter;
      }
      if (secCounter - lastReport >= REPORT_INTERVAL_MIN * 60)
      {
        Serial.printf("%d loop count in last %d seconds\n", loopCount, REPORT_INTERVAL_MIN * 60);
        Serial.printf("Current state: %d\n", state);
        Serial.printf("Box weight: %d\n", scale.getBoxWeight());
#if SEND_SNAPSHOT
        snapshotLoopCount[--snapeshotTimer] = loopCount;
        if (snapeshotTimer == 0)
        {
          uint32_t avg = snapshotLoopCount[0];
          uint32_t max = snapshotLoopCount[0];
          uint32_t min = snapshotLoopCount[0];
          for (int i = 1; i < SNAPSHOT_INTERVAL_MIN; i++)
          {
            max = (snapshotLoopCount[i] > max) ? snapshotLoopCount[i] : max;
            min = (snapshotLoopCount[i] < min) ? snapshotLoopCount[i] : min;
            avg += snapshotLoopCount[i];
          }
          avg = avg / SNAPSHOT_INTERVAL_MIN;
          char json [256];
          snprintf (json, sizeof(json),"{\"Type\":\"Snapshot\",\"LoopCnt Avg\":\"%d\", \"LoopCnt Max\":\"%d\", \"LoopCnt Min\":\"%d\", \"State\":\"%d\", \"Current boxWeight\":\"%d\", \"Current weight\":\"%d\"}", avg, max, min, state, scale.getBoxWeight(), scale.getWeight());
          logger.post_event(String(json));
          snapeshotTimer = SNAPSHOT_INTERVAL_MIN;
        }
#endif

        lastReport = secCounter;
        loopCount = 0;
      }
#endif
      oneSecInterval = true;
    }
}

inline void idle_func()
{
  if (xSemaphoreTake(lidSemaphore, 0) == pdTRUE)
  {
    detachInterrupt(LID_GPIO);
    state = clean;
  }
  else if ((currentSample_g - scale.getBoxWeight() > CAT_THRESH_G) && oneSecInterval) // A cat is inside the box! align to sec counter
  {
    state = deposit;
    depositTimerSec = 0;
    sampleCount = 0;
    steadyState = false;
  }
  else if (zeroingInterval)
  {
    check_zero();
  }
}

inline void check_zero()
{
  int32_t delta = scale.getBoxWeight() - scale.getWeight();
  if ((delta > MAX_ZEORING_DELTA_G) || (delta < -MAX_ZEORING_DELTA_G))
    scale.refreshOffset();
  zeroingInterval = false;
}

inline void deposit_func()
{
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

  if (oneSecInterval) // full second
  {
    depositTimerSec++;
    secSample_g[0] = secSample_g[0] / SPS;
    if ((secSample_g[0] - scale.getBoxWeight() < CAT_THRESH_G ) || (depositTimerSec == MAX_DEPOSIT_LEN_S)) //cat left or timeout
    {
      if (rfid.available())
      {
        catID = rfid.getLastTagRead();
        logger.event_catID(String(catID));
      }
      else 
      {
        logger.event_catID("Unknown");
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
      catWeight = windowAvg_g - scale.getBoxWeight();
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
        {
          steadyState = true;
          Serial.println("Steady state");
        }
      } 
    }
  }
}

inline void clean_func()
{
  if (!digitalRead(LID_GPIO)) //reading low -> lid closed
  {
    delay(1000); // debunce connection and verify
    if (!digitalRead(LID_GPIO)) //reading low -> lid closed
    {
      logger.event_sample(scale.getBoxWeight());
      scale.updateBoxWeight();
      logger.event_sample(scale.getBoxWeight());
      logger.event_type("Clean");
      logger.event_send();
      state = idle;
      rfid.restart();
      attachInterrupt(LID_GPIO, lidISR, RISING);
    }
  }
}

inline void rfid_only()
{
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
}
