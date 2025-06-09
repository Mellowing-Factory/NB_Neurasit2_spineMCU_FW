#ifndef _ALL_SENSORS_H
#define _ALL_SENSORS_H

#include "globals.h"
#include <ICM42670P.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include "customAlgorithms.h"
#include <math.h>


#define ADS1115_ADDRESS0 (0x48) ///< 1001 000 (ADDR = GND)
#define ADS1115_ADDRESS1 (0x49) ///< 1001 001 (ADDR = VDD)

#define ICM42670P_ADDRESS0 (0x68) ///< 1101 000 (AP_AD0 = GND)
#define ICM42670P_ADDRESS1 (0x69) ///< 1101 001 (AP_AD0 = VDD)
#define GYRO_FSR 500
#define GYRO_SR 50
#define ICM_BUFFER_LEN 10
#define TILT_DETECTION_DELAY_TIME 25
#define ACCEL_TILT_THRESHOLD 1500
#define ACCEL_SMOOTHNESS_THRESHOLD 800
#define GYRO_TILT_THREHOLD 1000
#define GYRO_SMOOTHNESS_THRESHOLD 400


void initAds1();
void initImu1();

void initAllSensors();

void measureImu1();
void measureAds1();

void initDeepSleepMode();


#endif