#include "customAlgorithms.h"

static const char TAG[] = __FILE__;

// breathing rate algorithm
// original array length 60s
// 1) find smoothing window average (window size = 12s, slowest breathing rate at 5breaths/min)
// 2) subtract result of 1 from original array in the middle (new array has length 48s)
// 3) find first zero crossing of new array (for loop to find element[i-1]<0 and element[i]>0)
// 4) find maximum of the array segment after the first zero crossing, and before the second zero crossing
// 5) repeat for all zero crossings with element[i] > 0
// 6) find the distance between the local maxima, get median distance, convert to breathing rate


uint8_t calculateBR(int16_t inputArray[ADS_BUFFER_LEN]) {
    

    int dummyHz = ADS_SAMPLING_HZ;

    int16_t maximum = -32765;
    int16_t minimum = 32765;
    int16_t range = 0;
    int smoothingFactor = dummyHz/2; // int(IMU_BUFFER_HZ*2 /2);      // to remove high frequency noise
    int decimationFactor = 1; // int(IMU_BUFFER_HZ/2); // apply decimation of approx 0.5s
    int smoothedArrayLength = (ADS_BUFFER_LEN)/decimationFactor;
    int16_t smoothedArray[smoothedArrayLength];         // middle 45s array is 100 samples long
    for (size_t i = 0; i < smoothedArrayLength; i++) {
		smoothedArray[i] = 0;
	}
    int newIndexCounter = smoothingFactor;
    
    for (size_t i=0; i<ADS_BUFFER_LEN; i+=decimationFactor) {
        Serial.printf("%d,", inputArray[i]);
    }
    Serial.println("");
    
    for (size_t i=0; i<ADS_BUFFER_LEN; i+=decimationFactor) {
        maximum = std::max(inputArray[i], maximum);
        minimum = std::min(minimum, inputArray[i]);
        if (i < smoothingFactor) {
            smoothedArray[i] = 0;
        }
        else if (i >= smoothingFactor && i < smoothedArrayLength - smoothingFactor) {
// smoothing between i-window to i+window
            int smoothedValue = 0;
            for (int j=i-smoothingFactor; j<=i+smoothingFactor; j++) {
                smoothedValue += inputArray[j];
            }
            smoothedValue = smoothedValue / (smoothingFactor*2);
            smoothedArray[newIndexCounter++] = smoothedValue; // - movingWindowValue;
        }
        else {
            smoothedArray[i] = 0;
        }
    }
    range = maximum - minimum;
    
// Creating a buffer to store the distances (maximum 30 per minute)
    uint8_t cyclesDistances[30];
    uint8_t cyclesCount = 0;
    for (int i=smoothingFactor+2; i<smoothedArrayLength-2; i++) {
        if (smoothedArray[i] > smoothedArray[i-1] && smoothedArray[i] >= smoothedArray[i-2] && smoothedArray[i] >= smoothedArray[i+1] && smoothedArray[i] >= smoothedArray[i+2]) {
			for (int j=i+1; j<smoothedArrayLength; j++) {
                if (smoothedArray[j] > smoothedArray[j-1] && smoothedArray[j] >= smoothedArray[j-2] && smoothedArray[j] >= smoothedArray[j+1] && smoothedArray[j] >= smoothedArray[j+2]) {
                    if ((j - i)*decimationFactor > 2*dummyHz && (j - i)*decimationFactor < 12*dummyHz) {
						cyclesDistances[cyclesCount++] = (j - i)*decimationFactor;
						// Serial.print(i);
						// Serial.print(", ");
						// Serial.println(j);	
						i = j;
						break;
					}
                }
            }
        }
    }

// getting a portion of the above buffer that is filled with values
    uint8_t cyclesDistances2[cyclesCount];
    for (int i=0; i<cyclesCount; i++) {
        cyclesDistances2[i] = cyclesDistances[i];
    }
    // Number of items in the array
    int lt_length = sizeof(cyclesDistances2) / sizeof(cyclesDistances2[0]);
    // qsort - last parameter is a function pointer to the sort function
    qsort(cyclesDistances2, lt_length, sizeof(cyclesDistances2[0]), sort_desc);

    float medianDistance = cyclesDistances2[int(cyclesCount/2)];
    float breathingRate = 0; //60/(medianDistance/dummyHz);
    if (medianDistance > 20) {
        breathingRate = 60/(medianDistance/dummyHz);
    }

    printf("max %d, min %d, range %d, range_uint8: %u\n", maximum, minimum, range, convert_int16_to_uint8(range));
    printf("Breathing rate: %.2f\n", breathingRate);

    return (uint8_t)breathingRate;
}

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

uint8_t convert_int16_to_uint8(int16_t value) {
    return (uint8_t)((float(value) / 65535.0) * 255);
    // return (uint8_t)((float(value) / 16384.0) * 255);
}

extern Kalman_t kalman;
float dt = 0.1f; // 100ms loop
uint8_t tilt_count = 0;
int tilted = 0; // 0 = neutral, 1 = tilted

void initKalman(Kalman_t* kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
}

float Kalman_Update(Kalman_t* kf, float newAngle, float newRate, float dt) {
    // Predict
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + 0.001f);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += 0.003f * dt;

    // Measurement update
    float S = kf->P[0][0] + 0.03f;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

void tilt_calculation(float ax, float ay, float az, float gyro_y) {
    // Estimate pitch from accelerometer
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // Gyro is in deg/sec
    float filtered_pitch = Kalman_Update(&kalman, accel_pitch, gyro_y, dt);

    // Threshold logic
    float TILT_THRESHOLD = 30.0f;

    if (fabs(filtered_pitch) > TILT_THRESHOLD && !tilted) {
        tilt_count++;
        tilted = 1;
    } else if (fabs(filtered_pitch) < 10.0f) {
        tilted = 0; // Reset state when close to neutral
    }

    // printf("Tilt: %.2f, Count: %d\n", filtered_pitch, tilt_count);
}
