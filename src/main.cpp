/************************************************************
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

Changed by O.bordes August 2019, add teapot output data

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

*************************************************************/
#include <Arduino.h>
#include <stdint.h>
#include <SparkFunMPU9250-DMP.h>
// estimator
#include "altitude.h"
#include "globals.h"
// Baro HP206C on SPI
#include <HP20x_dev.h>
#include <KalmanFilter.h>
#include <array>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "FreeRTOS.h"

#define ALTI_ESTIMATOR false
#define DEBUG_ENABLE true
#define CALIBRATE false // baro
#define USE_EWMA true

constexpr float GROUND_ALTI = 177.86;
constexpr float GROUND_PRESSURE = 992.07;

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
static void espDelay(int ms)
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

#define I2C_CTRL 1
#if I2C_CTRL == 0
#define WireInstance Wire
#define SDA 21
#define SCL 22
#define FREQ 400000L

#else 
#define WireInstance Wire1
#define SDA 27
#define SCL 26
#define FREQ 400000L

#endif
HP20x_dev HP20x(I2C_CTRL, SDA, SCL, FREQ);

#include <Ewma.h>
Ewma baroFilter1(0.1);   // Less smoothing - faster to detect changes, but more prone to noise
Ewma baroFilter2(0.01);  // More smoothing - less prone to noise, but slower to detect changes

// Number of readings from which standard deviations will be computed
constexpr int iterations = 1000;

typedef struct {
  float t;
  float tf;
  float p;
  float pf;
  float a;
  float af;
} baro_reading_t;
unsigned char hp206_available = 0;

#define TAG_MAIN "main"

/* Instance */

baro_reading_t g_baro_latest;

#define SerialPort Serial

#define DEBUG(format) SerialPort.printf(format, __FILE__, __LINE__)

#ifdef OUTPUT_READABLE_YAWPITCHROLL
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#endif

// Altitude estimator
static AltitudeEstimator altitude = AltitudeEstimator(
  0.000355, // sigma Accel 0.000354660293112
  0.000206, // sigma Gyro 0.000206332998559
  0.105356,   // sigma Baro 0.105355456471443
        0.5, // ca
        0.1);// accelThreshold

unsigned char ret = 0;

/* Instance */
grove::KalmanFilter t_filter;    //temperature filter
grove::KalmanFilter p_filter;    //pressure filter
grove::KalmanFilter a_filter;    //altitude filter

float pastTime = millis();
float currentTime = millis();


MPU9250_DMP imu;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
const signed char orientationMatrix[9] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};


float gravity_vect[3];

constexpr int DMP_FIFO_RATE = 5;
constexpr int ACCEL_FSR = 2;
constexpr int mpu_int_pin = 4;
constexpr int SELF_TEST_SUCCESS = 0x07;
constexpr float DECLINATION_NORTH = 20.0;

// helper variables and functions for obtaining baro data
static const uint8_t HISTORY_SIZE = 48;

static float   groundAltitude = 0;
static float   groundPressure = 0;
static float   pressureSum = 0;
static float   history[HISTORY_SIZE];
static uint8_t historyIdx = 0;
static uint32_t endCalibration = 120;


// Pressure in millibars to altitude in meters. We assume
// millibars are the units of the pressure readings from the sensor
static float millibarsToMeters(float mbar)
{
    // see: https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
    return (1.0f - powf(mbar / 1013.25f, 0.190295f)) * 44330.0f;
}
// Calibrate the baro by setting the average measured pressure as the
// ground pressure and the corresponding altitude as the ground altitude.
static void calibrate(float pressure)
{
    // Update pressure history
    history[historyIdx] = pressure;
    pressureSum += pressure;
    // cycle the index throught the history array
    uint8_t nextIndex = (historyIdx + 1) % HISTORY_SIZE;
    // Remove next reading from sum so that pressureSum is kept in sync
    pressureSum -= history[nextIndex];
    historyIdx = nextIndex;
    // groundPressure will stabilize at 8 times the average measured
    // pressure (when groundPressure/8 equals pressureSum/(HISTORY_SIZE-1))
    // This acts as a low pass filter and helps to reduce noise
    groundPressure -= groundPressure / 8;
    groundPressure += pressureSum / (HISTORY_SIZE - 1);
    groundAltitude = millibarsToMeters(groundPressure/8);
}
// void getPressure(float* p)
void getPressure(baro_reading_t& read)
{
  float pi =0.0;
  char display[40];
  if(OK_HP20X_DEV == ret)
  { 

	  long Temper = HP20x.ReadTemperature();
	  float t = Temper/100.0;
    read.t = t;
    auto tf = t_filter.Filter(t);
    read.tf = tf;

 
    long Pressure = HP20x.ReadPressure();
	  pi = Pressure/100.0;
    read.p = pi;
    auto pf = p_filter.Filter(pi);
    read.pf = pf;
	  
	  long Altitude = HP20x.ReadAltitude();
    float a = Altitude/100.0;
    read.a = a;

#if USE_EWMA == true
    auto af = baroFilter1.filter(a);
    auto af2 = baroFilter2.filter(a);
    read.af = af2;
  #if DEBUG_ENABLE
    Serial.printf("Raw Alti=%.3f, Filter1=%.3f, Filter2=%.3f", read.a, af, af2);
  #endif

#else
    auto af = a_filter.Filter(a);
    read.af = af;
#endif
  


#if DEBUG_ENABLE
	  Serial.println(F("------------------\n"));
    Serial.println(F("Temper:"));
	  Serial.print(t);	  
	  Serial.println(F("C.\n"));
	  Serial.println(F("Filter:"));
    Serial.print(tf);
	  Serial.println(F("C.\n"));

    Serial.println(F("Pressure:"));
    Serial.print(pi);
	  Serial.println(F("hPa.\n"));
	  Serial.println(F("Filter:"));
    Serial.print(pf);
	  Serial.println(F("hPa\n"));

	  Serial.println(F("Altitude:"));
	  Serial.print(a);
	  Serial.println(F("m.\n"));
	  Serial.println(F("Filter:"));
	  Serial.print(af);
	  Serial.println(F("m.\n"));
	  Serial.println(F("------------------\n"));
#endif

    // delay(50);
    }
}

void scanner(TwoWire& instance)
{
	 byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    instance.beginTransmission(address);
    error = instance.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(2000);  
}

void init_sensors()
{
  /* Reset HP20x_dev */
  HP20x.begin();
  delay(100);

  /* Determine HP20x_dev is available or not */
  bool found = false;
  while(!found)
  {
    ret = HP20x.isAvailable();
    if(OK_HP20X_DEV == ret)
    {
      Serial.println("HP20x_dev is available.\n");   
      delay(1000);
      found = true; 
    }
    else
    {
      Serial.println("HP20x_dev isn't available.\n");
      delay(1000);
      ret = HP20x.isAvailable();
    }
  }

   // Set all pressure history entries to 0
  for (uint8_t k = 0; k < HISTORY_SIZE; ++k) {
      history[k] = 0;
  }

  #if CALIBRATE
  // calibrate barometer
  uint32_t count = 0;
  while (count < endCalibration) 
  {
      baro_reading_t r;
      getPressure(r);
      calibrate(r.p);
      count++;
  }
#else
    groundAltitude = GROUND_ALTI;
    groundPressure = GROUND_PRESSURE;
#endif

#if DEBUG_ENABLE
  Serial.print("Ground pressure = ");
  Serial.println(groundPressure);

  Serial.print("Ground Altitude = ");
  Serial.println(groundAltitude);
#endif
  delay(5000);
}

float imu_GravityCompensatedAccel(float ax, float ay, float az, float* q)
{
  float za =  2.0f * (q[1] * q[3] - q[0] * q[2]) * ax 
            + 2.0f * (q[0] * q[1] + q[2] * q[3]) * ay +
             (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]) * az - 1.0f;
  return za;
}

void setup() 
{
  SerialPort.begin(115200);
  delay(200);
  SerialPort.println("Setup");

   // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()) {
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again
   
  WireInstance.begin(SDA, SCL, FREQ);
  scanner(WireInstance);
#if I2C_CTRL == 1
  Wire.begin(21, 22, FREQ);
  scanner(Wire);
#endif


  pinMode(mpu_int_pin, INPUT);
  digitalWrite(mpu_int_pin, LOW);

  init_sensors();

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  SerialPort.println("MPU-9250 init ok");
  if ( imu.setAccelFSR(ACCEL_FSR) != INV_SUCCESS)
  {
    DEBUG("Error: dmpSetOrientation() Failed! %s:%d\n");
    return;
  }

 
  int self_test_rc = imu.selfTest() ;
  if (self_test_rc != SELF_TEST_SUCCESS)
  {
    DEBUG("Error: selfTest() Failed! %s:%d\n");

    int gyro_success = self_test_rc & 0x01;
    int accel_success = self_test_rc & 0x02 ;
    int mag_success = self_test_rc & 0x04;

    printf("Error: gyro %d, accel %d, mag %d\n", gyro_success, accel_success, mag_success);
    return;
  }

  if ( imu.dmpSetOrientation(orientationMatrix) != INV_SUCCESS)
  {
    DEBUG("Error: dmpSetOrientation() Failed! %s:%d\n");
    return;
  }
 
  if ( imu.dmpEnableFeatures(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL) != INV_SUCCESS)
  {
    DEBUG("Error: dmpEnableFeatures() Failed! %s:%d\n");
    return;
  }

  if ( imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO, // Use gyro calibration 
              DMP_FIFO_RATE) != INV_SUCCESS )// Set DMP FIFO rate to 10 Hz
  {
    DEBUG("Error: dmpSetOrientation() Failed! %s:%d\n");
    return;
  }

  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  SerialPort.printf("MPU-9250 DMP ok DMP fifo rate = %d\n", DMP_FIFO_RATE);

  delay(3000);

//csv header
Serial.println("alti estimated veloc acc");


}          

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);


  float A31, A32, A33;   
  float lin_ax1, lin_ay1, lin_az1;      

  // calculate gravity vector
  gravity_vect[0] = 2 * (q1*q3 - q0*q2);
  A32 = gravity_vect[0];
  gravity_vect[1] = 2 * (q0*q1 + q2*q3);
  A31 = gravity_vect[1];
  gravity_vect[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  A33 = gravity_vect[2];

  float aRes = (float)ACCEL_FSR/32768.0;
  lin_ax1 = imu.calcAccel(imu.ax) + A31;
  lin_ay1 = imu.calcAccel(imu.ay) + A32;
  lin_az1 = imu.calcAccel(imu.az) - A33;

  // lin_ax1 = imu.ax * aRes + A31;
  // lin_ay1 = imu.ay * aRes + A32;
  // lin_az1 = imu.az * aRes - A33;

  // lin_ax1 = imu.ax * aRes ;
  // lin_ay1 = imu.ay * aRes ;
  // lin_az1 = imu.az * aRes ;

#if DEBUG_ENABLE == true
  SerialPort.println("lin acc xyz: " + String(lin_ax1) + ", "
            + String(lin_ay1) + ", " + String(lin_az1));
#endif
 

}

void loop() 
{
  currentTime = millis();
  unsigned long dmp_timestamp = 0; // DMP timestamp

  if ((currentTime - pastTime) > 100)
  {
    // Check for new data in the FIFO
    if ( imu.fifoAvailable() )
    {
      // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if ( imu.dmpUpdateFifo() == INV_SUCCESS)
      {
        dmp_timestamp = imu.time;
        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        // imu.computeEulerAngles();
        imu.update(UPDATE_COMPASS);

        baro_reading_t r; 
        getPressure(r);

        float alti = r.af - groundAltitude;
    #if DEBUG_ENABLE
        Serial.print("Ground Altitude: ");
        Serial.println(alti);
    #endif

        float accelDataGPerSec[3];
        accelDataGPerSec[0] = imu.calcAccel(imu.ax);
        accelDataGPerSec[1] = imu.calcAccel(imu.ay);
        accelDataGPerSec[2] = imu.calcAccel(imu.az);

    #if DEBUG_ENABLE
        Serial.print("aCCEL g/s XYZ ");
        Serial.print(accelDataGPerSec[0]);
        Serial.print("\t");
        Serial.print(accelDataGPerSec[1]);
        Serial.print("\t");
        Serial.print(accelDataGPerSec[2]);
        Serial.print("\t");
    #endif

        float gyroDataRadPerSec[3];
        gyroDataRadPerSec[0] = imu.calcGyro(imu.gx) * DEG_TO_RAD;
        gyroDataRadPerSec[1] = imu.calcGyro(imu.gy) * DEG_TO_RAD;
        gyroDataRadPerSec[2] = imu.calcGyro(imu.gz) * DEG_TO_RAD;

    #if DEBUG_ENABLE
        Serial.print("- Gyro rad/s XYZ \t");
        Serial.print(gyroDataRadPerSec[0]);
        Serial.print("\t");
        Serial.print(gyroDataRadPerSec[1]);
        Serial.print("\t");
        Serial.print(gyroDataRadPerSec[2]);
        Serial.print("\t");
        Serial.println(".");
    #endif

    #if DEBUG_ENABLE
        Serial.println("alti estimated veloc acc");
    #endif

    #if ALTI_ESTIMATOR == true
        altitude.estimate(accelDataGPerSec, gyroDataRadPerSec, alti, timestamp);
        Serial.print(alti);
        Serial.print(" ");
        Serial.print(altitude.getAltitude());
        Serial.print(" ");
        Serial.print(altitude.getVerticalVelocity());
        Serial.print(" ");
        Serial.println(altitude.getVerticalAcceleration());
    #endif
        printIMUData();
        delay(500);
      }
    }

    pastTime = currentTime;
  }
}


