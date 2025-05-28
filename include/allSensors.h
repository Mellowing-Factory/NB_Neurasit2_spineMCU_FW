#ifndef _ALL_SENSORS_H
#define _ALL_SENSORS_H

#include "globals.h"
#include <ICM42670P.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_ADS1X15.h>


#define ADS1115_ADDRESS0 (0x48) ///< 1001 000 (ADDR = GND)
#define ADS1115_ADDRESS1 (0x49) ///< 1001 001 (ADDR = VDD)

#define ICM42670P_ADDRESS0 (0x68) ///< 1101 000 (AP_AD0 = GND)
#define ICM42670P_ADDRESS1 (0x69) ///< 1101 001 (AP_AD0 = VDD)
#define GYRO_FSR 500
#define GYRO_SR 50

#define TOF_ADDRESS0 (0x29)

extern All_data_t allData; 
extern PredictionDataClass predictionData;

void initAds1();
void initAds2();
void initAds3();
void initAds4();

void initImu0();
void initImu1();
void initImu2();
void initImu3();
void initImu4();

void initTof();

void initAllSensors();

void measureTof();
void measureImu1();
void measureImu2();
void measureImu3();
void measureImu4();
void measureAds1();
void measureAds2();
void measureAds3();
void measureAds4();

#endif