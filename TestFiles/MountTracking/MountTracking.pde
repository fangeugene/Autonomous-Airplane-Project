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

#include <AAP_IRCamera.h>
#include <AAP_Mount.h>
#include <I2C.h>

#include <AP_Math.h>
#include <matrix3.h>
#include <polygon.h>
#include <vector2.h>
#include <vector3.h>

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
static unsigned long 	medium_loopCounter = 0;
// Number of milliseconds used in last main loop cycle
static uint8_t 		delta_ms_fast_loop;

AAP_IRCamera IRCamera;
AAP_Mount Mount;
static void flash_leds(bool on)
{
  digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
  digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

template <typename T>
void printVector(Vector3<T> v)
{
  Serial.printf("%4.2f  %4.2f  %4.2f\n", v.x, v.y, v.z);
}

template <typename T>
void printMatrix(Matrix3<T> m)
{
  printVector(m.a);
  printVector(m.b);
  printVector(m.c);
}

float angle(Vector3f Va, Vector3f Vb, Vector3f Vn)
{
  float sina = (Va%Vb).length();
  float cosa = Va*Vb;
  float angle = atan2( sina, cosa );
  if (Vn*(Va%Vb) < 0.0)
	angle *= -1.0;
  return angle;
}

void setup(void)
{
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  Serial.begin(9600);
  Serial.println("Doing IMU startup...");
  
  isr_registry.init();
  timer_scheduler.init(&isr_registry);
  APM_RC.Init(&isr_registry);
  
  imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
  dcm.matrix_reset();
  
  IRCamera.init();
  Mount.init(&APM_RC, 0, 1);
  
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
    
    /*Serial.printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
      (int)dcm.roll_sensor / 100,
      (int)dcm.pitch_sensor / 100,
      (uint16_t)dcm.yaw_sensor / 100,
      gyros.x, gyros.y, gyros.z,
      accels.x, accels.y, accels.z);*/
    
    float yaw = (float)dcm.yaw_sensor / 100.0;
    float pitch = (float)dcm.pitch_sensor / 100.0;
    float roll = (float)dcm.roll_sensor / 100.0;
    Serial.printf(" YPR %3.2f %3.2f %3.2f END\n", yaw, pitch, roll);
    
    Matrix3f rotation = Matrix3f(-ToRad(yaw), Vector3f(0.0, 0.0, 1.0))
                        * Matrix3f(ToRad(pitch), Vector3f(1.0, 0.0, 0.0))
                        * Matrix3f(ToRad(roll), Vector3f(0.0, 1.0, 0.0));
    
    Vector2i ir_pos_raw[4];
    IRCamera.getRawData(ir_pos_raw);
    Serial.printf(" IRPOS %i %i %i %i %i %i %i %i END\n", ir_pos_raw[0].x, ir_pos_raw[0].y,
                                                          ir_pos_raw[1].x, ir_pos_raw[1].y,
                                                          ir_pos_raw[2].x, ir_pos_raw[2].y,
                                                          ir_pos_raw[3].x, ir_pos_raw[3].y);
    Vector3f pos;
    IRCamera.getTransform2(pos, rotation);
    Serial.printf(" POS ");
    printVector(pos);
    Serial.printf(" END ");
    Serial.printf(" ROT ");
    printMatrix(rotation);
    Serial.printf(" END ");

    //Update mount
    Matrix3f mount_rotation = Matrix3f(ToRad(180.0), rotation.col(2)) * rotation;
    
    vector<Vector2f> ir_pos;
    for (int i=0; i<4; i++)
      if ((ir_pos_raw[i].x != 1023) && (ir_pos_raw[i].y != 1023))
	ir_pos.push_back(Vector2f((float) ir_pos_raw[i].x, (float) ir_pos_raw[i].y));
    Vector2f mean_ir_pos;
    for (int i=0; i<4; i++)
      mean_ir_pos += ir_pos[i];
    mean_ir_pos /= ir_pos.size();
    Vector2f ccd_center(512,384);
    float focal_length = 1280;
    Vector2f off_center = mean_ir_pos - ccd_center;
    Vector3f track = mount_rotation * Vector3f(off_center.x, focal_length, off_center.y);
    track.normalize();
    
    Mount.update(mount_rotation, track);
    Serial.printf(" TRACK %3.2f %3.2f %3.2f END\n", track.x, track.y, track.z);
    Serial.printf(" PANTILT %3.2f %3.2f END\n", Mount.getPan(), Mount.getTilt());
  }
}
