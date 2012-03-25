// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_IMU_Oilpan driver.
//

#include <FastSerial.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <SPI.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_IMU.h>         // ArduPilot Mega IMU Library
#include <AP_DCM.h>         // ArduPilot Mega DCM Library
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_ADC.h>
#include <APM_RC.h> // ArduPilot Mega RC Library
APM_RC_APM1 APM_RC;

// Configuration
#include "config.h"

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
FastSerialPort(Serial, 0);

////////////////////////////////////////////////////////////////////////////////
// ISR Registry
////////////////////////////////////////////////////////////////////////////////
Arduino_Mega_ISR_Registry isr_registry;

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
// All GPS access should be through this pointer.
static GPS *g_gps;
AP_GPS_None g_gps_driver(NULL);

static AP_ADC_ADS7844          adc;
//AP_InertialSensor_MPU6000 mpu6000( 53 ); /* chip select is pin 53 */
AP_InertialSensor_Oilpan ins( &adc );
//AP_IMU_INS imu(&mpu6000, 0); /* second arg is for Parameters. Can we leave it null?*/
AP_IMU_INS imu(&ins, 0);
AP_DCM dcm(&imu, g_gps);

// we always have a timer scheduler
AP_TimerProcess timer_scheduler;

////////////////////////////////////////////////////////////////////////////////
// LED output
////////////////////////////////////////////////////////////////////////////////
#define A_LED_PIN        37
#define C_LED_PIN        35
#define LED_ON           LOW
#define LED_OFF          HIGH

////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt						= 0.02;		

////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in miliseconds of start of main control loop.  Milliseconds
static unsigned long 	fast_loopTimer;
// Number of milliseconds used in last main loop cycle
static uint8_t 		delta_ms_fast_loop;

static void flash_leds(bool on)
{
  digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
  digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

/** Returns the mapped integer of "value". This linear map F is defined such that:
  * F(value_min) = mapped_min
  * F(value_max) = mapped_max
  **/
int linearMap(int value, int value_min, int value_max, int mapped_min, int mapped_max) {
  return mapped_min + (((float)(value - value_min))/((float)(value_max-value_min)))*(mapped_max-mapped_min);
}

/** The same as linear_map except that if the mapped integer is outside the range 
  * [mapped_min, mapped_max], then the mapped integer is clipped to either mapped_min
  * or mapped max, whichever is the closest to the mapped integer.
  **/
int linearMapClip(int value, int value_min, int value_max, int mapped_min, int mapped_max) {
  int mapped = linearMap(value, value_min, value_max, mapped_min, mapped_max);
  if (mapped_min < mapped_max) {
    if (mapped < mapped_min)
      mapped = mapped_min;
    else if (mapped > mapped_max)
      mapped = mapped_max;
  } else {
    if (mapped < mapped_max)
      mapped = mapped_max;
    else if (mapped > mapped_min)
      mapped = mapped_min;
  }
  return mapped;
}

void setup(void)
{
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  Serial.begin(115200);
  Serial.println("Doing IMU startup...");
  
  isr_registry.init();
  timer_scheduler.init(&isr_registry);
  APM_RC.Init(&isr_registry);
  
  imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
  dcm.matrix_reset();
  
  delay(1000);
}

void loop(void)
{
  delay(20);
  if (millis() - fast_loopTimer > 19) {
    delta_ms_fast_loop 	= millis() - fast_loopTimer;
    G_Dt 		= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
    fast_loopTimer	= millis();
    
    // IMU
    // ---
    dcm.update_DCM();
    
    // We are using the IMU
    // ---------------------
    Vector3f gyros = imu.get_gyro();
    Vector3f accels = imu.get_accel();
    
    Serial.printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
      (int)dcm.roll_sensor / 100,
      (int)dcm.pitch_sensor / 100,
      (uint16_t)dcm.yaw_sensor / 100,
      gyros.x, gyros.y, gyros.z,
      accels.x, accels.y, accels.z);
    
    int yaw = (uint16_t)dcm.yaw_sensor / 100;
    // The function linearMapClip below maps yaw values to servoValue values
    // yaw        servo arm angle      servoValue                             _
    // 0          -45                  2000            _                       | clipped region
    // 135        -45                  2000             |                     _|
    // 135 + 45     0                  1500             | linear map region   _
    // 135 + 90    45                  1000            _|                      | clipped region
    // 359         45                  1000                                   _|
    int servoValue = linearMapClip(yaw, 135, 135+90, 2000, 1000);
    
    Serial.printf("Yaw: %d  Servo 0 Value: %d\n", yaw, servoValue);
    APM_RC.OutputCh(0, servoValue);
  }
}
