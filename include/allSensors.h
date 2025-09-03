#ifndef _ALL_SENSORS_H
#define _ALL_SENSORS_H

#include "globals.h"
#include <ICM42670P.h>
#include <Wire.h>
#include "customAlgorithms.h"
#include <math.h>

#define ICM42670P_ADDRESS0 (0x68) ///< 1101 000 (AP_AD0 = GND)
#define ICM42670P_ADDRESS1 (0x69) ///< 1101 001 (AP_AD0 = VDD)
#define GYRO_FSR 500
#define GYRO_SR 50
#define ACCEL_FSR 4

void initAllSensors();
void initImu0();
void initImu1();
void initImu2();
void initImu3();
void measureImu0();
void measureImu1();
void measureImu2();
void measureImu3();


#endif