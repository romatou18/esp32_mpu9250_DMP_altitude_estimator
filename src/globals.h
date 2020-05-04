#pragma once

#include <stdint.h>

#define TEST_MODE
/**********************************************
 * SCREEN
 **********************************************
 */
#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23

#define TFT_BL          4  // Display backlight control pin
#define ADC_EN          14
#define ADC_PIN         34

/**********************************************
 * SPI
 **********************************************
 */
#define SD_MOSI 26
#define SD_MISO 25
#define SD_SCKL 32
#define SD_SPEED 4000000U //26 Mhz max on matrixed HSPI otherwise 27mhz

/**********************************************
 * SD CARD
 **********************************************
 */
#define SD_CS 33

/**********************************************
 * BUTTONS
 **********************************************
 */
#define BUTTON_1        35
#define BUTTON_2        0

#define FORMAT_SPIFFS_IF_FAILED true

/**********************************************
 * GPS
 **********************************************
 */
#define GPS_REFRESH_RATE_HZ 10
#define GPS_PPS_INT_ENABLED 1
static constexpr uint32_t GPSBaud = 9600U;
static constexpr int RX_pin = 38, TX_pin = 37;
static constexpr int GPS_PPS_PIN = 39;
static constexpr int GPS_QUEUE_SIZE = 20;


/**********************************************
 * BAROMETER / I2C #1
 **********************************************
 */
static constexpr uint8_t refresh_rate_baro_hz = 15;
static constexpr uint8_t I2CBUS_ID_HP206 = 0;
#define SDA_1 21
#define SCL_1 22
static constexpr uint8_t BARO_QUEUE_SIZE = 30;
#define BARO_TAG "baro"


/**********************************************
 * MPU9250 / I2C #1
 **********************************************
 */

// #define MPU_HIDEAKITAI false
// #define MPU_BOLDER true
// #define MPU_SPARKFUN trues

static constexpr float NELSON_MAGNETIC_DECLINATION = 20.49;
#define MPU9250_I2C_ADR 0x68
// #define I2CBUS_ID_MPU9250 0
#define SDA_2 17
#define SCL_2 2
#define IMU_CS 13
#define IMU_INT_PIN 12 
#define WAKE_ON_MOTION_INTERRUPT_PIN 12

#define IMU_QUEUE_SIZE 400

typedef enum
{
  GPS_T,
  BARO_T,
  IMU_T
} EventType_t;

typedef struct 
{
  void *update_p;
  EventType_t type;
} QueueUpdate;


/***************************
 * TASKS DELAYS
 ***************************/
#define GPS_TASK_DELAY 1U
#define SCREEN_TASK_DELAY 10U

#define GPS_PERIOD 100
#define SCREEN_UPDATE_PERIOD 50
#define IMU_PERIOD 10
#define BARO_PERIOD 65
#define LOG_PERIOD 20

/*********************************
 * Task scheduler config
 *********************************/
// // #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
// #define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass 
#define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
#define _TASK_PRIORITY          // Support for layered scheduling priority
// // #define _TASK_MICRO_RES         // Support for microsecond resolution
// // #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE       // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT           // Support for overall task timeout 