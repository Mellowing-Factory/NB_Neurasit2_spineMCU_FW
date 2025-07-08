#include "allSensors.h"

static const char TAG[] = __FILE__;

extern All_data_t allData; 
Adafruit_ADS1115 ads1; // left strap front
extern SPIClass mySPI;
ICM42670 IMU1(mySPI, CS_PIN); // left strap front

bool imu_valid1 = false;
int handler_delay = 1000/GYRO_SR;
#ifdef BLE_TEST
    int16_t ble_test_int16t_value1 = 0;
#endif

int dummy = 0;
volatile uint8_t irq_received = 0;
uint32_t step_count = 0;
uint16_t sec_counter = 0;
float step_cadence = 0;
const char* activity = "IDLE";
uint8_t gait_state = 0;

int16_t adsBuffer[ADS_BUFFER_LEN];
uint16_t adsIndex = 0;
uint8_t breathingRate = 0;
const int BUFFER_SIZE = 6;
std::vector<uint8_t> breathingRateBuffer;
uint8_t breathingRateSmoothed;

uint8_t walkTimeSec = 0;

uint16_t isIdleCounter = 0;

float max_accel = 0;
float min_accel = 1000000;

// uint8_t tilt_delay_counter = 0;
// bool sec_window_passed = false;
// bool tiltDetected = false;

extern portMUX_TYPE timerMux2;
extern portMUX_TYPE timerMux3;
extern volatile int interruptCounter2;
extern volatile int interruptCounter3;
int tenSecondCounter = 0;

void IRAM_ATTR icm_irq_handler() {
    // Serial.println("IRQ received!");
    irq_received = 1;
}

void allSensorsHandler(void *pvParameters) {
    while (1) {

        // unsigned long startTime = millis();

        measureImu1();
        measureAds1();

        // unsigned long endTime = millis();
        // // Calculate the time difference in microseconds
        // unsigned long duration = endTime - startTime;
        // // Print the result
        // Serial.print("Function took ");
        // Serial.print(duration);
        // Serial.println(" milli seconds to execute.");

        vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

void initAllSensors() {

    setupBreathingRateBuffer(breathingRateBuffer);

    delay(75);
	initAds1();
    delay(75);
	initImu1();

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

void initImu1() {
    delay(100);
    // Serial.println("00000 ");
    int ret1;
    ret1 = IMU1.begin();
    // Serial.println("1111111 ");
    // Initializing the ICM42670P
    if (ret1 != 0) {
        Serial.print("IMU1 ICM42670P SPI initialization failed: ");
        Serial.println(ret1);
        while(ret1 != 0) {
            ret1 = IMU1.begin();
            if (ret1 != 0) {
                Serial.print("IMU1 ICM42670P SPI initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

#ifdef ACCEL
    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU1.startAccel(GYRO_SR, ACCEL_FSR);
#endif
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU1.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);

    // Pedometer enabled
    IMU1.startPedometer();

// // NOTE: native tilt detection has >2s delay, it is used to detect if the devices' position completely changed
//     // Tilt enabled
//     IMU1.startTiltDetection();

    delay(100);
// NOTE: for some reason ICM_INT1 pin works but not ICM_INT2, i think you can choose to use one or the other
    // Enable interrupt
    IMU1.enableInterrupt(INT1_PIN, icm_irq_handler);

    ESP_LOGI(TAG, "IMU1 initialized...");
}

void initAds1() {
    delay(100);

    ads1.setGain(GAIN_ONE);
    ads1.setDataRate(RATE_ADS1115_32SPS);

    bool ret1;
    ret1 = ads1.begin(ADS1115_ADDRESS0, &Wire1);

    if (ret1 != true) {
        ESP_LOGE(TAG, "Failed to initialize ADS1.");
        while(ret1 != true) {
            ret1 = ads1.begin(ADS1115_ADDRESS0, &Wire1);
            if (ret1 != true) {
                ESP_LOGE(TAG, "Failed to initialize ADS1.");
            }
            else {
                break;
            }
            vTaskDelay(100/ portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "ADS1 initialized...");
}

void measureImu1() {
    inv_imu_sensor_event_t imu_event1;

    imu_valid1 = IMU1.isGyroDataValid(&imu_event1);
    if (imu_valid1) {
        // Get last event
        IMU1.getDataFromRegisters(imu_event1);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU1.getDataFromRegisters(imu_event1);
    }

#ifndef BLE_TEST  
#ifdef ACCEL
    allData.M_accel1[0] = imu_event1.accel[0];
    allData.M_accel1[1] = imu_event1.accel[1];
    allData.M_accel1[2] = imu_event1.accel[2];
#endif
    allData.M_gyro1[0] = imu_event1.gyro[0];
    allData.M_gyro1[1] = imu_event1.gyro[1];
    allData.M_gyro1[2] = imu_event1.gyro[2];
#else
    allData.M_accel2[0] = ble_test_int16t_value1;
    allData.M_accel2[1] = ble_test_int16t_value1;
    allData.M_accel2[2] = ble_test_int16t_value1;
    allData.M_gyro2[0] = ble_test_int16t_value1;
    allData.M_gyro2[1] = ble_test_int16t_value1;
    allData.M_gyro2[2] = ble_test_int16t_value1;
    ble_test_int16t_value1 += 256;
    if (ble_test_int16t_value1 > 65536) {
        ble_test_int16t_value1 = 0;
    }
#endif
    
    if (irq_received) {
        // Serial.println("IMU IRQ received!");
        irq_received = 0;
        dummy = IMU1.getPedometer(step_count, step_cadence, activity);
        if (strcmp(activity, "WALK") == 0) { gait_state = 1; }
        else if (strcmp(activity, "RUN") == 0) { gait_state = 2; }
        // else { gait_state = 0; }  // this resets gait_state too often and walkTimeSec is not increased
    }

    if (interruptCounter2 > 0) {
        portENTER_CRITICAL(&timerMux2);
        interruptCounter2--;
        portEXIT_CRITICAL(&timerMux2);
        
        float accel_x_g = imu_event1.accel[0] / 8192.0f;  // 2g range → 16384 LSB/g
        float accel_y_g = imu_event1.accel[1] / 8192.0f;  // 16g range -> 2048
        float accel_z_g = imu_event1.accel[2] / 8192.0f;  // 4g range -> 8192 LSB/g
        float gyro_y_dps = imu_event1.gyro[1] / 65.5f;    // 250 dps range → 131 LSB/(°/s)
        tilt_calculation(accel_x_g, accel_y_g, accel_z_g, gyro_y_dps);
        
        // printf("accelX: %.2f, accelY: %.2f, accelZ: %.2f\n", accel_x_g, accel_y_g, accel_z_g);
        float sum_accel = fabs(accel_x_g) + fabs(accel_y_g) + fabs(accel_z_g);
        max_accel = std::max(sum_accel, max_accel);
        min_accel = std::min(min_accel, sum_accel);

        sec_counter += 1;
        if (sec_counter >= 10*STEP_DETECT_PERIOD) {
            if (gait_state == 1) {
                walkTimeSec += 1*STEP_DETECT_PERIOD;
            }
            else if (gait_state == 2) { 
                walkTimeSec += 2*STEP_DETECT_PERIOD;
            }
            // Serial.printf("walk time seconds: %u\n", walkTimeSec);
            sec_counter = 0;
            Serial.printf("activity: %s", activity);
            Serial.printf(", state: %u\n", gait_state);

            // by resetting gait_state here, we can guarantee that any walk/run is retained for over a sec
            // dummy = IMU1.getPedometer(step_count, step_cadence, activity);
            // if (strcmp(activity, "WALK") == 0) { gait_state = 1; }
            // else if (strcmp(activity, "RUN") == 0) { gait_state = 2; }
            // else { gait_state = 0; } 
            activity = "IDLE";
            gait_state = 0;
        }
    }
}

void measureAds1() {
#ifndef BLE_TEST        
    allData.M_ads1 = ads1.continuous_readADC_SingleEnded(2);
#else
    allData.M_ads1 = ble_test_int16t_value2;
    ble_test_int16t_value2 += 256;
    if (ble_test_int16t_value2 > 65536) {
        ble_test_int16t_value2 = 0;
    }
#endif
    if (interruptCounter3 > 0) {
        portENTER_CRITICAL(&timerMux3);
        interruptCounter3--;
        portEXIT_CRITICAL(&timerMux3);

        if (adsIndex == ADS_BUFFER_LEN) {
            for (int i = 0; i < ADS_BUFFER_LEN-1; i++) {
                adsBuffer[i] = adsBuffer[i+1];
            }
            adsIndex -= 1;
        }
        adsBuffer[adsIndex] = allData.M_ads1;
        adsIndex += 1;
        tenSecondCounter += 1;
    }

    // calculate breathing rate if user has been stable for more than 10seconds
    if (tenSecondCounter >= 100) {
        printf("min accel: %.2f, max accel: %.2f\n", min_accel, max_accel);;
        for (size_t i=0; i<ADS_BUFFER_LEN-1; i+=1) {
            Serial.printf("%d,", adsBuffer[i]);
        }
        Serial.println("");

        
        if (max_accel < 3 && max_accel-min_accel < 2) {
            breathingRate = calculateBR(adsBuffer);
        }
        else {
            breathingRate = 0;
        }
        addBreathingRate(breathingRate, breathingRateBuffer);
        breathingRateSmoothed = getAverageBreathingRate(breathingRateBuffer);
        Serial.print("Smoothed Breathing Rate: ");
        Serial.println(breathingRateSmoothed);

        // Serial.print("Current Buffer: [");
        // for (size_t i = 0; i < breathingRateBuffer.size(); ++i) {
        //     Serial.print(breathingRateBuffer[i]);
        //     if (i < breathingRateBuffer.size() - 1) {
        //         Serial.print(", ");
        //     }
        // }
        // Serial.println("]");


        // enter deep sleep if the device is idle for more than 10min  
        if (max_accel < 1.4 && max_accel-min_accel < 0.05) {
            isIdleCounter += 1;
            if (isIdleCounter > 60) {
                initDeepSleepMode();
            }
        }
        else {
            isIdleCounter = 0;
        }
        // printf("%d\n", isIdleCounter);

        tenSecondCounter = 0;
        max_accel = 0;
        min_accel = 1000000;
    }

}

void IRAM_ATTR int1_isr() {
	Serial.println("INT1 pin interrupt generated");
}

void initDeepSleepMode() {
	turnLedOff(LED_PIN_GR);
    pinMode(INT1_PIN, INPUT_PULLDOWN);
    int pinState = digitalRead(INT1_PIN);
    Serial.printf("INT1 pin status: %d\n", pinState);
    delay(100);
    int ret1;
    ret1 = IMU1.startWakeOnMotion(INT1_PIN, int1_isr);
    Serial.print("IMU start WOM: ");
    Serial.println(ret1);
    esp_sleep_enable_ext0_wakeup(INT1_PIN, 1);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    delay(100);
    pinState = digitalRead(INT1_PIN);
    ESP_LOGI(TAG, "INT1 pin status: %d", pinState);
    Serial.println(" ");
	ESP_LOGI(TAG, "Going to deep sleep");
    Serial.println(" ");
 
    delay(100);
    esp_deep_sleep_start();
}
