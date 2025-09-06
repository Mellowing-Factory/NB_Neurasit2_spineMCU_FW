#include "customAlgorithms.h"

static const char TAG[] = __FILE__;

extern Kalman_t kalman0;
extern Kalman_t kalman1;
extern Kalman_t kalman2;
extern Kalman_t kalman3;
float dt = 0.05f; // 100ms loop

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

float tilt_calculation(float ax, float ay, float az, float gyro_y, int kalmanNum) {
    // Estimate pitch from accelerometer
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    float filtered_pitch = 0;
    // Gyro is in deg/sec
    if (kalmanNum == 0) {
        filtered_pitch = Kalman_Update(&kalman0, accel_pitch, gyro_y, dt);
    }
    else if (kalmanNum == 1) {
        filtered_pitch = Kalman_Update(&kalman1, accel_pitch, gyro_y, dt);
    }
    else if (kalmanNum == 2) {
        filtered_pitch = Kalman_Update(&kalman2, accel_pitch, gyro_y, dt);
    }
    else if (kalmanNum == 3) {
        filtered_pitch = Kalman_Update(&kalman3, accel_pitch, gyro_y, dt);
    }
    // printf("Kalman Pitch: %.2f\n", filtered_pitch);

    return filtered_pitch;
}