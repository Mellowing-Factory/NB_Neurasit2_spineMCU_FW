#ifndef _CUSTOM_ALGORITHMS_H
#define _CUSTOM_ALGORITHMS_H

#include "globals.h"
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <vector> // For std::vector
#include <numeric> // For std::accumulate

/* Test Settings */
#ifdef DEV_MODE
    #ifdef TEST_MODE
        // #define ALGORITHM_TEST
    #endif
#endif

#define BR_BUFFER_SIZE 6

uint8_t calculateBR(int16_t inputArray[ADS_BUFFER_LEN]);  // breathing rate, motion range
int sort_desc(const void *cmp1, const void *cmp2);
uint8_t convert_int16_to_uint8(int16_t value);
void tilt_calculation(float ax, float ay, float az, float gyro_y);

typedef struct {
    float angle; // The filtered angle
    float bias;
    float rate;
    float P[2][2];
} Kalman_t;

void initKalman(Kalman_t* kf);

void setupBreathingRateBuffer();
void addBreathingRate(uint8_t newRate);
uint8_t getAverageBreathingRate(std::vector<uint8_t> breathingRateBuffer);

#endif
