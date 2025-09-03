#ifndef _CUSTOM_ALGORITHMS_H
#define _CUSTOM_ALGORITHMS_H

#include "globals.h"
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include <vector> // For std::vector
#include <numeric> // For std::accumulate


typedef struct {
    float angle; // The filtered angle
    float bias;
    float rate;
    float P[2][2];
} Kalman_t;

void initKalman(Kalman_t* kf);
void setupBreathingRateBuffer(std::vector<uint8_t>& breathingRateBuffer);
float tilt_calculation(float ax, float ay, float az, float gyro_y);

#endif
