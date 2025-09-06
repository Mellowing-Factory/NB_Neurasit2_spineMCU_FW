#include "allSensors.h"

static const char TAG[] = __FILE__;

ICM42670 IMU0(Wire, 0);
ICM42670 IMU1(Wire, 1);
ICM42670 IMU2(Wire1, 0);
ICM42670 IMU3(Wire1, 1);

extern All_data_t allData;
extern bool sendData;
bool calcAngles = false;

bool imu_valid = false;
int handler_delay = 1000/GYRO_SR;

void allSensorsHandler(void *pvParameters) {
    while (1) {
        if (calcAngles) {
            measureImu0();
            measureImu1();
            measureImu2();
            measureImu3();
            sendData = true;
            calcAngles = false;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void initAllSensors() {

    delay(75);
	initImu0();
    delay(75);
	initImu1();
    delay(75);
	initImu2();
    delay(75);
	initImu3();
    delay(75);

    xTaskCreatePinnedToCore(allSensorsHandler,
                            "allSensorsHandler",
                            SENSORS_TASK_STACK,
                            NULL,
                            SENSORS_TASK_PRI,
                            &TaskSensors_h,
                            SENSORS_TASK_CORE);
    ESP_LOGI(TAG, "All I2C sensors initialized...");
    delay(75);

}

void initImu0() {
    delay(100);
    int ret1;
    ret1 = IMU0.begin();
    // Serial.println("1111111 ");
    if (ret1 != 0) {
        Serial.print("IMU0 initialization failed: ");
        Serial.println(ret1);
        while(ret1 != 0) {
            ret1 = IMU1.begin();
            if (ret1 != 0) {
                Serial.print("IMU0 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU0.startAccel(GYRO_SR, ACCEL_FSR);
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU0.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);

    ESP_LOGI(TAG, "IMU0 initialized...");
}

void measureImu0() {
    inv_imu_sensor_event_t imu_event0;
    imu_valid = IMU0.isGyroDataValid(&imu_event0);
    if (imu_valid) {
        // Get last event
        IMU0.getDataFromRegisters(imu_event0);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU0.getDataFromRegisters(imu_event0);
    }

    float accel_x_g = imu_event0.accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
    float accel_y_g = imu_event0.accel[1] / 8192.0f;  // 16g range -> 2048
    float accel_z_g = imu_event0.accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
    float gyro_x_dps = imu_event0.gyro[0] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
    float gyro_y_dps = imu_event0.gyro[1] / 65.5f;
    // allData.roll0 = roll_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, 0);
    allData.pitch0 = pitch_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_y_dps, 0);
}

void initImu1() {
    delay(100);
    int ret1;
    ret1 = IMU1.begin();
    // Serial.println("1111111 ");
    if (ret1 != 0) {
        Serial.print("IMU1 initialization failed: ");
        Serial.println(ret1);
        while(ret1 != 0) {
            ret1 = IMU1.begin();
            if (ret1 != 0) {
                Serial.print("IMU1 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU1.startAccel(GYRO_SR, ACCEL_FSR);
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU1.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);

    ESP_LOGI(TAG, "IMU1 initialized...");
}

void measureImu1() {
    inv_imu_sensor_event_t imu_event1;
    imu_valid = IMU1.isGyroDataValid(&imu_event1);
    if (imu_valid) {
        // Get last event
        IMU1.getDataFromRegisters(imu_event1);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU1.getDataFromRegisters(imu_event1);
    }

    float accel_x_g = imu_event1.accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
    float accel_y_g = imu_event1.accel[1] / 8192.0f;  // 16g range -> 2048
    float accel_z_g = imu_event1.accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
    float gyro_x_dps = imu_event1.gyro[0] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
    float gyro_y_dps = imu_event1.gyro[1] / 65.5f;
    // allData.roll1 = roll_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, 1);
    allData.pitch1 = pitch_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_y_dps, 1);
}

void initImu2() {
    delay(100);
    int ret1;
    ret1 = IMU2.begin();
    // Serial.println("1111111 ");
    if (ret1 != 0) {
        Serial.print("IMU2 initialization failed: ");
        Serial.println(ret1);
        while(ret1 != 0) {
            ret1 = IMU2.begin();
            if (ret1 != 0) {
                Serial.print("IMU2 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU2.startAccel(GYRO_SR, ACCEL_FSR);
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU2.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);

    ESP_LOGI(TAG, "IMU2 initialized...");
}

void measureImu2() {
    inv_imu_sensor_event_t imu_event2;
    imu_valid = IMU2.isGyroDataValid(&imu_event2);
    if (imu_valid) {
        // Get last event
        IMU2.getDataFromRegisters(imu_event2);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU2.getDataFromRegisters(imu_event2);
    }

    float accel_x_g = imu_event2.accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
    float accel_y_g = imu_event2.accel[1] / 8192.0f;  // 16g range -> 2048
    float accel_z_g = imu_event2.accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
    float gyro_x_dps = imu_event2.gyro[0] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
    float gyro_y_dps = imu_event2.gyro[1] / 65.5f;
    // allData.roll2 = roll_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, 2);
    allData.pitch2 = pitch_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_y_dps, 2);
}

void initImu3() {
    delay(100);
    int ret1;
    ret1 = IMU3.begin();
    // Serial.println("1111111 ");
    if (ret1 != 0) {
        Serial.print("IMU3 initialization failed: ");
        Serial.println(ret1);
        while(ret1 != 0) {
            ret1 = IMU3.begin();
            if (ret1 != 0) {
                Serial.print("IMU3 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU3.startAccel(GYRO_SR, ACCEL_FSR);
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU3.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);

    ESP_LOGI(TAG, "IMU3 initialized...");
}

void measureImu3() {
    inv_imu_sensor_event_t imu_event3;
    imu_valid = IMU3.isGyroDataValid(&imu_event3);
    if (imu_valid) {
        // Get last event
        IMU3.getDataFromRegisters(imu_event3);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU3.getDataFromRegisters(imu_event3);
    }

    float accel_x_g = imu_event3.accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
    float accel_y_g = imu_event3.accel[1] / 8192.0f;  // 16g range -> 2048
    float accel_z_g = imu_event3.accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
    float gyro_x_dps = imu_event3.gyro[0] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
    float gyro_y_dps = imu_event3.gyro[1] / 65.5f;
    // allData.roll3 = roll_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, 3);
    allData.pitch3 = pitch_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_y_dps, 3);
}