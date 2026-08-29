#include <Arduino.h>
#include <Preferences.h>
#include <algorithm>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "SPI.h"
#include "ADS1256.h"
#include "RFID.h"
#include "logger.h"

// #define NO_SCALE
// #define SCALE_DEBUG
#define DEBUG_PRINTS 1
#define DEBUG 0
#define SEND_SNAPSHOT 1

#define LID_GPIO 11
#define USER_BUTTON 0
#define SETTING_HOLD_THRESH_SEC 3 // seconds to hold user button in order to enter remote setting mode
#define REMOTE_SETTINGS_TIMEOUT_SEC 30 // max time to wait for remote setting reply
#define TICKS_PER_SEC 1000000
#define SAMPLE_PRINT_INTERVAL_SEC 2
#define SNAPSHOT_INTERVAL_MIN 15
#define REPORT_INTERVAL_MIN 1
#define ZEROING_INTERVAL_SEC 30
#define MIN_ZEROING_DELTA_G 2
#define MAX_ZEROING_DELTA_G 15
#define MAX_DEPOSIT_WEIGHT_G 250
#define MIN_DEPOSIT_TIME_SEC 10

#define SPS 100
#define SAMPLE_BUFFER_SEC 8
#define SAMPLE_BUFFER_SIZE (SAMPLE_BUFFER_SEC * SPS)
#define CAT_THRESH_G 1500
#define MAX_DEPOSIT_LEN_SEC 300
#define MAX_SAMPLES_BETWEEN_TRANSIENT_PARTS (SPS / 5) // transient should be less than 0.2 sec
#define RESETS_FOR_MEM_CLEAR 5 //num of resets within window to issue a clear cmd
#define RESET_WINDOW_MS 5000 // max time between consecutive resets such that they would count towards memory clear
#define WATCHDOG_TIMEOUT_MINUTES 14
#define WATCHDOG_TIMEOUT_SEC (WATCHDOG_TIMEOUT_MINUTES * 60)
#define BLOCKS_PER_SEC 10
#define SAMPLES_PER_BLOCK (SPS / BLOCKS_PER_SEC)
#define NO_MOVEMENT_THRESH_SEC 4
#define DEPOSIT_END_THRESH_SEC (NO_MOVEMENT_THRESH_SEC * 2) // require box to be quite for long time before declaring deposit is over
#define MAX_NO_MOVEMENT_MAD 30

//Impulse detection params
#define CANDIDATE_OFFSET_SEC NO_MOVEMENT_THRESH_SEC // only after 2 quite seconds do we start looking for impulses
#define BASELINE_SIZE_SAMP 55 //~550ms before and after each candidate
#define CANDIDATE_SIZE_SAMP 11
#define POO_IMPULSE_THRESH_G 120
#define CANDIDATE_SURROUNDING_MAD_MAX_G 25
#define IMPULSE_RATIO_THRESH 10
#define TRANSIENT_CNT_THRESH 3
#define IMPULSE_COOLDOWN_SAMP 30
#define CANDIDATE_CURRENT_SECOND_MASK (((uint32_t)1) << (CANDIDATE_OFFSET_SEC - 1))

#define LED_TOGGLE digitalWrite(LED_BUILTIN, HIGH); delay(500); digitalWrite(LED_BUILTIN, LOW);

Preferences stateNVM;

enum FsmState
{
  idle,         // 0
  deposit,      // 1 
  clean,        // 2
  test          // 3
};

enum DepositPhase
{
  preElimination, // 0
  elimination,     // 1 
  postElimination // 2
};

volatile SemaphoreHandle_t oneSecondSemaphore;
volatile SemaphoreHandle_t lidSemaphore;
volatile uint32_t secCounter = 0;

static FsmState state = idle;
static DepositPhase depositPhase = preElimination;

static uint32_t depositTimerSec = 0;
static uint32_t loopCount = 0;
static uint32_t snapshotLoopCount[SNAPSHOT_INTERVAL_MIN / REPORT_INTERVAL_MIN] = {0};
static uint32_t snapshotTimer = SNAPSHOT_INTERVAL_MIN / REPORT_INTERVAL_MIN;
static uint32_t sampleCount = 0;
static uint32_t samplesThisSecond = 0;
static uint32_t impulseCooldown = 0;
static uint32_t lastSamplePrint = 0;
static uint32_t lastReport = 0;
static uint32_t lastZero = 0;
static int32_t currentSample_g = 0;
static int32_t previousSample_g = 0;
static uint32_t sampleBufferHead = 0;
static int32_t sampleBuffer[SAMPLE_BUFFER_SIZE] = {0};
static int32_t blockAvg[BLOCKS_PER_SEC] = {0};
static int32_t blockAbsDeviation[BLOCKS_PER_SEC] = {0};
static int32_t oneSecondMAD = 0;
static int32_t oneSecondMax = 0;
static int32_t oneSecondMin = 0;
static int32_t oneSecondMedian[NO_MOVEMENT_THRESH_SEC] = {0};
static int32_t baselineSamples[2 * BASELINE_SIZE_SAMP];
static uint32_t candidateStartIndex;
static uint32_t preBaselineStartIndex;
static uint32_t postBaselineStartIndex;
static int32_t impulseCandMax;
static int32_t impulseCandMin;
static int32_t baseline;
static int32_t localMAD;
static int32_t candP2P;
static int32_t depositWeight = 0;
static int32_t catWeightSamplesCount = 0;
static int32_t catWeight = 0;
static int32_t quietSec = 0;
static int32_t pooTransientCnt = 0;
static bool quietPeriodLax = false;
static bool quietPeriodStrict = false;
static bool zeroingInterval = false;
static bool oneSecondTick  = false;
static bool pooDetected = false;
static bool isLidClosed = true;
Logger logger;
ADS1256 scale;
rfidReader rfid(Serial1);
static const char *catId;
static bool clearMem   = false;
static bool clearCred  = false;
static bool clearScale = false;
hw_timer_t *oneSecondTimer = NULL;

void remoteSettings();
void tareScale();
void checkUserButton();
void userButtonFunc();
void periodicals();
void idleFunc();
void depositFunc();
void finalizeDeposit(bool overlength = false);
void cleanFunc();
void rfidOnly();
void scaleFsm();
void checkZero();
void calculateOneSecondStats();

void IRAM_ATTR oneSecondISR()
{
  secCounter++;
  xSemaphoreGiveFromISR(oneSecondSemaphore, NULL);
  return;
}

void IRAM_ATTR lidISR()
{
  xSemaphoreGiveFromISR(lidSemaphore, NULL);
  return;
}

void setup()
{
  const esp_reset_reason_t reason = esp_reset_reason();
  const bool watchdogReset = ((reason == ESP_RST_TASK_WDT) || (reason == ESP_RST_INT_WDT) || (reason == ESP_RST_WDT));
  stateNVM.begin("resetsNvm", false);
  int32_t resetCount = stateNVM.getInt("rstCount");
  stateNVM.putInt("rstCount", resetCount + 1);
  delay(RESET_WINDOW_MS);
  clearMem = false;
  #if DEBUG
    // clearMem = true;
    state= test;
  #endif
  
  if (resetCount >= RESETS_FOR_MEM_CLEAR)
  {
    clearMem = true;
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
  
  if (clearMem)
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
          clearCred = true;
          Serial.println("WiFi credentials will be CLEARED!");
          break;
        }
        else if (res.startsWith("N"))
        {
          clearCred = false;
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
          clearScale = true;
          Serial.println("Scale parameters will be CLEARED!");
          break;
        }
        else if (res.startsWith("N"))
        {
          clearScale = false;
          Serial.println("Will NOT clear scale pararmeters");
          break;
        }
      }
    }
    res.clear();
  }
  logger.initLogger(clearCred);
  rfid.begin();
#ifndef NO_SCALE
  SPI.begin(SCALE_CLK, SCALE_MISO, SCALE_MOSI);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  scale.begin(clearScale);
  Serial.println("ADS1256 Configured, starting sampling loop");
#endif

  if (watchdogReset)
  {
    logger.event_type("Boot - watchdog");
  }
  else
  {
    logger.event_type("Boot - normal");
  }
  logger.event_sample(0);
  logger.event_send();
  
  lidSemaphore = xSemaphoreCreateBinary();
  pinMode(USER_BUTTON, INPUT_PULLUP);
  pinMode(LID_GPIO, INPUT_PULLUP);
  attachInterrupt(LID_GPIO, lidISR, RISING);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  oneSecondSemaphore = xSemaphoreCreateBinary();
  esp_timer_init();
  oneSecondTimer = timerBegin(0, (APB_CLK_FREQ / TICKS_PER_SEC), true);
  timerAttachInterrupt(oneSecondTimer, &oneSecondISR, true);
  timerAlarmWrite(oneSecondTimer, TICKS_PER_SEC, true);
  timerAlarmEnable(oneSecondTimer);
  snapshotTimer = SNAPSHOT_INTERVAL_MIN / REPORT_INTERVAL_MIN;

  ESP_ERROR_CHECK(esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true));
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
}

void loop()
{
  checkUserButton();
  periodicals();
#ifndef NO_SCALE
  scaleFsm();
#else// #ifndef NO_SCALE
  rfidOnly();
#endif
  loopCount++;
}

inline void scaleFsm()
{
  if (scale.available())
  {
    sampleBuffer[sampleBufferHead] = previousSample_g;
    sampleBufferHead = (sampleBufferHead + 1) % SAMPLE_BUFFER_SIZE;
    previousSample_g = currentSample_g;
    currentSample_g = scale.getWeight();
    samplesThisSecond++;
    if (oneSecondTick)
    {
      calculateOneSecondStats();
      if (samplesThisSecond != SPS)
      {/*flag problem*/}
      samplesThisSecond = 0;
    }
#ifdef SCALE_DEBUG
    delay(1000); // reduce loop rate to 1SPS
    Serial.printf("current sample: %d grams, %d counts\n", currentSample_g, scale.getSample());
#else
    switch (state)
    {
      case idle:
        idleFunc();
        break;
      case deposit:
        depositFunc();
        break;
      case clean:
        cleanFunc();
        break;
      case test:
        if (oneSecondTick)
        {
          logger.event_sample(currentSample_g);
          logger.event_type("debug-test");
          logger.event_send();
        }
        state = test;
        break;
    }
#endif // #ifdef SCALE_DEBUG
    oneSecondTick = false;
  }
}

inline bool sendRemoteCmd(rcommand cmd, int32_t arg = 0)
{
  int32_t timeout = REMOTE_SETTINGS_TIMEOUT_SEC;
  while(timeout--)
  {
    if (logger.remote_setting(cmd, arg) != 0)
      break;
    delay(1000);
  }
  return ((timeout == 0) ? false : true);
}

void remoteSettings()
{
  Serial.println("Entered remote settings mode");
  if (!sendRemoteCmd(start))
    return;
  Serial.println("Offset cal");
  int32_t offset = scale.getSample();
  if (!sendRemoteCmd(req_tare, offset))
    return;
  scale.setManOffset(offset);
  delay(100);
  if (!sendRemoteCmd(ack_tare, scale.getSample() - offset))
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

inline void checkUserButton()
{
  if (!digitalRead(USER_BUTTON))
  {
    Serial.println("userbutton pressed");
    int32_t holdCnt = 0;
    for (;holdCnt < SETTING_HOLD_THRESH_SEC * 10; holdCnt++) // check every 100ms
    {
      delay(100);
      if(digitalRead(USER_BUTTON))
        break;
    }
    if (holdCnt == SETTING_HOLD_THRESH_SEC * 10)
      userButtonFunc();
  }
}

inline void userButtonFunc()
{
  detachInterrupt(LID_GPIO);
  // remoteSettings();
  LED_TOGGLE
  delay(500);
  tareScale();
  state = idle;
  attachInterrupt(LID_GPIO, lidISR, RISING);
}

inline void tareScale()
{
  while(!digitalRead(USER_BUTTON)) //wait for button release
  {
    delay(100);
  }
  delay(1000);
  while (true)
  {
    if(scale.available())
    {
      scale.setManOffset(scale.getSample());
      break;
    }
    delay(50);
  }
  while (true)
  {
    if(scale.available())
      break;
    delay(50);
  }
  int32_t baseWeight = scale.getWeight(); //should be ~0
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
    if (!digitalRead(LID_GPIO))
    {
      delay(2000);
      Serial.println("Lid closed, updating box weight");
      while(true)
      {
        if(scale.available())
        {
          scale.updateBoxWeight(scale.getWeight()); // lazy solution, no averaging/settling
          break;
        }
        delay(50);
      }
      Serial.println("Box weight updated, sending event");
      logger.event_type("scale_tare");
      logger.event_sample(baseWeight);
      logger.event_sample(scale.getBoxWeight());
      logger.event_send();
      LED_TOGGLE
      delay(500);
      LED_TOGGLE
      state = idle;
      break;
    }
  }
  Serial.println("Event sent, resuming normal operation");
}

inline void periodicals()
{
  if (xSemaphoreTake(oneSecondSemaphore, 0) == pdTRUE)
    {
      if (rfid.available())
      {
        catId = rfid.getLastTagRead();
        logger.event_catID(String(catId));
      }

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
        Serial.printf("Actuall SPS: %d\n", samplesThisSecond + 1);
#if SEND_SNAPSHOT
        snapshotLoopCount[--snapshotTimer] = loopCount;
        if (snapshotTimer == 0)
        {
          uint32_t avg = snapshotLoopCount[0];
          uint32_t max = snapshotLoopCount[0];
          uint32_t min = snapshotLoopCount[0];
          for (size_t i = 1; i < SNAPSHOT_INTERVAL_MIN / REPORT_INTERVAL_MIN; i++)
          {
            max = (snapshotLoopCount[i] > max) ? snapshotLoopCount[i] : max;
            min = (snapshotLoopCount[i] < min) ? snapshotLoopCount[i] : min;
            avg += snapshotLoopCount[i];
          }
          avg = avg / (SNAPSHOT_INTERVAL_MIN / REPORT_INTERVAL_MIN);
          char json [256];
          snprintf (json, sizeof(json),"{\"Type\":\"Snapshot\",\"LoopCnt Avg\":\"%d\", \"LoopCnt Max\":\"%d\", \"LoopCnt Min\":\"%d\", \"State\":\"%d\", \"Current boxWeight\":\"%d\", \"Current weight\":\"%d\"}", avg, max, min, state, scale.getBoxWeight(), scale.getWeight());
          logger.post_event(String(json));
          snapshotTimer = SNAPSHOT_INTERVAL_MIN / REPORT_INTERVAL_MIN;
        }
#endif

        lastReport = secCounter;
        loopCount = 0;
      }
#endif
      oneSecondTick = true;
    }
}

inline void idleFunc()
{
  ESP_ERROR_CHECK(esp_task_wdt_reset());

  if (xSemaphoreTake(lidSemaphore, 0) == pdTRUE)
  {
    delay(500); // debounce
    if(digitalRead(LID_GPIO)) //verify lid is open
    {
      detachInterrupt(LID_GPIO);
      isLidClosed = false;
      state = clean;
    }
  }
  else if ((currentSample_g - scale.getBoxWeight() > CAT_THRESH_G) && oneSecondTick) // A cat is inside the box! align to sec counter
  {
    state = deposit;
    depositPhase = preElimination;
    quietSec = 0;
    depositTimerSec = 0;
    sampleCount = 0;
    pooDetected = false;
    for (size_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
      logger.event_sample(sampleBuffer[(sampleBufferHead + i) % SAMPLE_BUFFER_SIZE]);
    }
  }
  else if (zeroingInterval)
  {
    checkZero();
  }
  else
    rfid.available(); // clear misreads
}

inline void checkZero()
{
  int32_t delta = scale.getBoxWeight() - currentSample_g;
  if (((delta > 0) && (delta < MAX_ZEROING_DELTA_G) && (delta > MIN_ZEROING_DELTA_G)) || ((delta < 0) && (delta > -MAX_ZEROING_DELTA_G) && (delta < -MIN_ZEROING_DELTA_G)))
    scale.refreshOffset();
  zeroingInterval = false;
}

inline void depositFunc()
{
  candidateStartIndex         = (sampleBufferHead + ((SAMPLE_BUFFER_SEC - CANDIDATE_OFFSET_SEC)* SPS)) % SAMPLE_BUFFER_SIZE;
  preBaselineStartIndex  = (candidateStartIndex + ((SAMPLE_BUFFER_SEC * SPS) - BASELINE_SIZE_SAMP))   % SAMPLE_BUFFER_SIZE;
  postBaselineStartIndex = (candidateStartIndex + CANDIDATE_SIZE_SAMP)                           % SAMPLE_BUFFER_SIZE;
  impulseCandMax  = INT32_MIN;
  impulseCandMin  = INT32_MAX;
  baseline = 0;
  localMAD = 0;
  candP2P = 0;
  sampleCount++;
  logger.event_sample(currentSample_g);
  if (oneSecondTick) // full second
  {
    depositTimerSec++;
  }
  if (depositTimerSec >= MAX_DEPOSIT_LEN_SEC)
  {
    finalizeDeposit(true);
    state = idle;
    return;
  }
  switch (depositPhase)
  {
    case preElimination:
      if (oneSecondTick && quietPeriodStrict) //no movement
      {
        if (oneSecondMedian[0] - scale.getBoxWeight() > CAT_THRESH_G) // got quite and cat's inside
        {
          catWeightSamplesCount = 0;
          catWeight = 0;
          pooTransientCnt = 0;
          impulseCooldown = 0;
          depositPhase = elimination;
        }
        else // got quite but cat left - summerize deposite
        {
          depositPhase = postElimination;
        }
      }
    // wait for quiet. of weight drop - abort
      break;
    case elimination:
      // test for poo transient
      if (!impulseCooldown)
      {
        for (size_t i = 0; i < CANDIDATE_SIZE_SAMP; i++)
        {
          if (impulseCandMax < sampleBuffer[(candidateStartIndex + i) % SAMPLE_BUFFER_SIZE])
          {
            impulseCandMax = sampleBuffer[(candidateStartIndex + i) % SAMPLE_BUFFER_SIZE];
          }
          if (impulseCandMin > sampleBuffer[(candidateStartIndex + i) % SAMPLE_BUFFER_SIZE])
          {
            impulseCandMin = sampleBuffer[(candidateStartIndex + i) % SAMPLE_BUFFER_SIZE];
          }
        }
        candP2P = impulseCandMax - impulseCandMin;
        if (candP2P > POO_IMPULSE_THRESH_G)
        {
          //calculate baseline stats
          for (size_t i = 0; i < BASELINE_SIZE_SAMP; i++)
          {
            baselineSamples[i] = sampleBuffer[(preBaselineStartIndex + i) % SAMPLE_BUFFER_SIZE];
            baselineSamples[i + BASELINE_SIZE_SAMP] = sampleBuffer[(postBaselineStartIndex + i) % SAMPLE_BUFFER_SIZE];
          }
          std::sort(baselineSamples, baselineSamples + (2 * BASELINE_SIZE_SAMP));
          baseline = (baselineSamples[BASELINE_SIZE_SAMP - 1] + baselineSamples[BASELINE_SIZE_SAMP]) / 2;
          for (size_t i = 0; i < BASELINE_SIZE_SAMP; i++)
          {
            baselineSamples[i] = (baselineSamples[i] >= baseline) ? baselineSamples[i] - baseline : baseline - baselineSamples[i];
            baselineSamples[BASELINE_SIZE_SAMP + i] = (baselineSamples[BASELINE_SIZE_SAMP + i] >= baseline) ? baselineSamples[BASELINE_SIZE_SAMP + i] - baseline : baseline - baselineSamples[BASELINE_SIZE_SAMP + i];
          }
          std::sort(baselineSamples, baselineSamples + (2 * BASELINE_SIZE_SAMP));
          localMAD = (baselineSamples[BASELINE_SIZE_SAMP - 1] + baselineSamples[BASELINE_SIZE_SAMP]) / 2;
          // compare with thresholds
          if (localMAD < CANDIDATE_SURROUNDING_MAD_MAX_G)
          {
            float ratio = (float)candP2P / (float)max(localMAD, 1);
            if (ratio > IMPULSE_RATIO_THRESH) // impulse detected!
            {
              pooTransientCnt++;
              impulseCooldown = IMPULSE_COOLDOWN_SAMP;
              logger.event_sample(-1000); // mark transiant
              // determine poo qualification
              if (pooTransientCnt >= TRANSIENT_CNT_THRESH)
              {
                pooDetected = true;
              }
            }
          }
        }
        else // low p2p
        {
          if (oneSecondTick && (quietSec & CANDIDATE_CURRENT_SECOND_MASK))
          {
            catWeight+=oneSecondMedian[CANDIDATE_OFFSET_SEC - 1];
            catWeightSamplesCount++;
          }
        } 
      }
      else
      {
        impulseCooldown--;
      }

      if (!quietPeriodLax || ((abs(oneSecondMedian[0] - scale.getBoxWeight()) < CAT_THRESH_G) && (abs(currentSample_g - scale.getBoxWeight()) < CAT_THRESH_G))) // movement or cat left
      {
        depositPhase = postElimination;
      }
      break;
    case postElimination:
      if (oneSecondTick && quietPeriodStrict) //no movement
      {
        if (oneSecondMedian[0] - scale.getBoxWeight() > MAX_DEPOSIT_WEIGHT_G) // cat still inside TODO: verify it's the same cat!
        {
          pooTransientCnt = 0;
          impulseCooldown = 0;
          depositPhase = elimination;
        }
        else if (quietSec >= DEPOSIT_END_THRESH_SEC) // box is empty for a while
        {
          finalizeDeposit();
          state = idle;
        }
      }
      break;
  }
}

void calculateOneSecondStats()
{
  oneSecondMax = INT32_MIN;
  oneSecondMin = INT32_MAX;
  for (size_t i = NO_MOVEMENT_THRESH_SEC - 1; i > 0; i--)
  {
    oneSecondMedian[i] = oneSecondMedian[i - 1];
  }
  for (size_t i = 0; i < BLOCKS_PER_SEC; i++)
  {
    blockAvg[i] = 0;
    for (size_t j = 0; j < SAMPLES_PER_BLOCK; j++)
    {
      blockAvg[i] += sampleBuffer[(sampleBufferHead + (((SAMPLE_BUFFER_SEC - 1) * SPS) + (i * SAMPLES_PER_BLOCK)) + j) % SAMPLE_BUFFER_SIZE];
    }
    blockAvg[i] = blockAvg[i] / SAMPLES_PER_BLOCK;
    oneSecondMax = (blockAvg[i] > oneSecondMax) ? blockAvg[i] : oneSecondMax;
    oneSecondMin = (blockAvg[i] < oneSecondMin) ? blockAvg[i] : oneSecondMin;
  }
  std::sort(blockAvg, blockAvg + BLOCKS_PER_SEC);
  oneSecondMedian[0] = (blockAvg[(BLOCKS_PER_SEC / 2) - 1] + blockAvg[BLOCKS_PER_SEC / 2]) / 2;
  for (size_t i = 0; i < BLOCKS_PER_SEC; i++)
  {
    blockAbsDeviation[i] = (blockAvg[i] >= oneSecondMedian[0]) ? blockAvg[i] - oneSecondMedian[0] : oneSecondMedian[0] - blockAvg[i];
  }
  std::sort(blockAbsDeviation, blockAbsDeviation + BLOCKS_PER_SEC);
  oneSecondMAD = (blockAbsDeviation[(BLOCKS_PER_SEC / 2) - 1] + blockAbsDeviation[BLOCKS_PER_SEC / 2]) / 2;

  quietSec <<= 1; // advance buffer
  if (oneSecondMAD < MAX_NO_MOVEMENT_MAD)
  {
    quietSec++;
  }

  //asses period
  quietPeriodStrict = true;
  quietPeriodLax = true;
  for (size_t i = 0; i < NO_MOVEMENT_THRESH_SEC; i++)
  {
    if (!((quietSec >> i) & (uint32_t)1))
    {
      quietPeriodLax = quietPeriodStrict;
      quietPeriodStrict = false;
    }
  }
}

void finalizeDeposit(bool overlength)
{
  if (depositTimerSec > MIN_DEPOSIT_TIME_SEC)
  {
    if (catWeightSamplesCount > 1)
    {
      catWeight = catWeight / catWeightSamplesCount;
    }
    if (!overlength)
    {
      if (pooDetected)
        logger.event_type("2");
      else
        logger.event_type("1");
      depositWeight = oneSecondMedian[0] - scale.getBoxWeight(); //current - previous
      logger.event_depositWeight(String(depositWeight));
      catWeight = catWeight - oneSecondMedian[0]; // catWeight prelodad with avg during deposit, subtract current box weight
      scale.updateBoxWeight(oneSecondMedian[0]);
    }
    else
    {
      logger.event_type("over_length");
    }
    logger.event_catWeight(String(catWeight));
  }
  else
  {
    logger.event_type("Aborted");
  }
  logger.event_send();
}

inline void cleanFunc()
{
  if (!isLidClosed)
  {
    if (!digitalRead(LID_GPIO)) //reading low -> lid closed
    {
      delay(1000); // debunce connection and verify
      if (!digitalRead(LID_GPIO)) //reading low -> lid closed
      {
        isLidClosed = true;
        quietSec = 0;
      }
    }
  }

  if(oneSecondTick && isLidClosed)
  {
    if(quietPeriodStrict)
    {
      logger.event_sample(scale.getBoxWeight());
      scale.updateBoxWeight(oneSecondMedian[0]);
      logger.event_sample(oneSecondMedian[0]);
      logger.event_type("Clean");
      logger.event_send();
      rfid.restart();
      state = idle;
      attachInterrupt(LID_GPIO, lidISR, RISING);
    }
  }
}

inline void rfidOnly()
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
    catId = rfid.getLastTagRead();
    logger.event_catID(String(catId));
    logger.event_type("rfid_only");
    logger.event_send();
  }
}
