#include "allSensors.h"

static const char TAG[] = __FILE__;

Adafruit_VL53L0X tofSensor = Adafruit_VL53L0X();
int16_t TOF_val = 0;

ICM42670 IMU1(Wire1, 0); // left strap front
bool imu_valid1 = false;
ICM42670 IMU2(Wire, 0); // right strap front
bool imu_valid2 = false;
ICM42670 IMU3(Wire1, 1); // left strap back (I2C address = 1)
bool imu_valid3 = false;
ICM42670 IMU4(Wire, 1); // right strap back (I2C address = 1)
bool imu_valid4 = false;
int handler_delay = 1000/GYRO_SR;
int handler_delay2 = 1000/GYRO_SR*2;
#ifdef BLE_TEST
    int16_t ble_test_int16t_value1 = 0;
#endif

Adafruit_ADS1115 ads1; // left strap front
Adafruit_ADS1115 ads2; // right strap front
Adafruit_ADS1115 ads3; // left strap back (I2C address = 1)
Adafruit_ADS1115 ads4; // right strap back (I2C address = 1)

VL53L0X_RangingMeasurementData_t tofMeasure;

// uint16_t predictionBufferIndex = 0;

void PredictionDataClass::resetBuffers(void) {
    
    memset(ADS1_buffer, 0, sizeof(ADS1_buffer));  // Set all elements to 0
    memset(IMU1_x_buffer, 0, sizeof(IMU1_x_buffer)); 
    memset(IMU1_y_buffer, 0, sizeof(IMU1_y_buffer)); 
    memset(IMU1_z_buffer, 0, sizeof(IMU1_z_buffer)); 
        
    memset(ADS3_buffer, 0, sizeof(ADS3_buffer));  // Set all elements to 0
    memset(IMU3_x_buffer, 0, sizeof(IMU3_x_buffer)); 
    memset(IMU3_y_buffer, 0, sizeof(IMU3_y_buffer)); 
    memset(IMU3_z_buffer, 0, sizeof(IMU3_z_buffer)); 

    memset(TOF_buffer, 0, sizeof(TOF_buffer)); 
    memset(dummy, 0, sizeof(dummy)); 

    index = 0;  
}

uint16_t PredictionDataClass::addData(All_data_t allData) {
    // if (index > BUFFER_LEN) { index = BUFFER_LEN; }

    if (index > BUFFER_LEN) {
        for (int i = 0; i < BUFFER_LEN - 1; i++) {
            ADS3_buffer[i] = ADS3_buffer[i + 1];
            IMU3_x_buffer[i] = IMU3_x_buffer[i + 1];
            IMU3_y_buffer[i] = IMU3_y_buffer[i + 1];
            IMU3_z_buffer[i] = IMU3_z_buffer[i + 1];
            ADS1_buffer[i] = ADS1_buffer[i + 1];
            IMU1_x_buffer[i] = IMU1_x_buffer[i + 1];
            IMU1_y_buffer[i] = IMU1_y_buffer[i + 1];
            IMU1_z_buffer[i] = IMU1_z_buffer[i + 1];
            TOF_buffer[i] = TOF_buffer[i + 1];
        }
    }    
    
    ADS1_buffer[index] = allData.M_ads1;
    IMU1_x_buffer[index] = allData.M_gyro1[0];
    IMU1_z_buffer[index] = allData.M_gyro1[1];
    IMU1_y_buffer[index] = allData.M_gyro1[2];
    
    ADS3_buffer[index] = allData.M_ads3;
    IMU3_x_buffer[index] = allData.M_gyro3[0];
    IMU3_z_buffer[index] = allData.M_gyro3[1];
    IMU3_y_buffer[index] = allData.M_gyro3[2];

    // Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d", ADS1_buffer[index], IMU1_x_buffer[index], IMU1_y_buffer[index], IMU1_z_buffer[index], ADS3_buffer[index], IMU3_x_buffer[index], IMU3_y_buffer[index], IMU3_z_buffer[index], TOF_buffer[index]);

    // Serial.println(allData.M_tof);
    TOF_buffer[index] = allData.M_tof;

    Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", ADS1_buffer[index], IMU1_x_buffer[index], IMU1_y_buffer[index], IMU1_z_buffer[index], ADS3_buffer[index], IMU3_x_buffer[index], IMU3_y_buffer[index], IMU3_z_buffer[index], TOF_buffer[index]);
    index += 1;

    return index;
}

void allSensorsHandler(void *pvParameters) {
    while (1) {

        // unsigned long startTime = millis();

        measureTof();
        measureImu1();
        // measureImu2();
        measureImu3();
        // measureImu4();
        measureAds1();
        // measureAds2();
        measureAds3();
        // measureAds4();

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
    delay(75);
	initTof();
    delay(75);
	initAds1();
    delay(75);
	initImu1();
    // delay(75);
	// initAds2();
    // delay(75);
	// initImu2();
    delay(75);
	initAds3();
    delay(75);
	initImu3();
    // delay(75);
	// initAds4();
    // delay(75);
	// initImu4();
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


void initTof() {
    delay(100);
    
    bool ret1;
    ret1 = tofSensor.begin(0x29, false, &Wire1);
    if (!ret1)
    {
        Serial.print("TOF I2C0 initialization failed: ");
        Serial.println(ret1);
        while(!ret1) {
            ret1 = tofSensor.begin();
            if (!ret1) {
                Serial.println("TOF I2C0 initialization failed ");
            }
            else {
                break;
            }
            vTaskDelay(100/ portTICK_PERIOD_MS);
        }
    }
    tofSensor.startRangeContinuous();
    ESP_LOGI(TAG, "TOF initialized...");
}

void initImu1() {
    delay(100);
    // Serial.println("00000 ");
    int ret1;
    ret1 = IMU1.begin();
    // Serial.println("1111111 ");
    // Initializing the ICM42670P
    if (ret1 != 0) {
        Serial.print("IMU1 ICM42670P I2C1 initialization failed: ");
        Serial.println(ret1);
        while(ret1 != 0) {
            ret1 = IMU1.begin();
            if (ret1 != 0) {
                Serial.print("IMU1 ICM42670P I2C1 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

#ifdef ACCEL
    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU1.startAccel(GYRO_SR, 16);
#endif
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU1.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);
    ESP_LOGI(TAG, "IMU1 initialized...");
}

void initImu2() {
    delay(100);
    int ret2;
    ret2 = IMU2.begin();
    // Initializing the ICM42670P
    if (ret2 != 0) {
        Serial.print("IMU2 ICM42670P I2C0 initialization failed: ");
        Serial.println(ret2);
        while(ret2 != 0) {
            ret2 = IMU2.begin();
            if (ret2 != 0) {
                Serial.print("IMU2 ICM42670P I2C0 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

#ifdef ACCEL
    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU2.startAccel(GYRO_SR, 16);
#endif
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU2.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);
    ESP_LOGI(TAG, "IMU2 initialized...");
}

void initImu3() {
    delay(100);
    int ret3;
    ret3 = IMU3.begin();
    // Initializing the ICM42670P
    if (ret3 != 0) {
        Serial.print("IMU3 ICM42670P I2C1 initialization failed: ");
        Serial.println(ret3);
        while(ret3 != 0) {
            ret3 = IMU3.begin();
            if (ret3 != 0) {
                Serial.print("IMU3 ICM42670P I2C1 initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

#ifdef ACCEL
    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU3.startAccel(GYRO_SR, 16);
#endif
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU3.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);
    ESP_LOGI(TAG, "IMU3 initialized...");
}

void initImu4() {
    delay(100);
    int ret4;
    ret4 = IMU4.begin();
    
    // Initializing the ICM42670P
    if (ret4 != 0) {
        Serial.print("IMU4 ICM42670P I2C0 initialization failed: ");
        Serial.println(ret4);
        while(ret4 != 0) {
            ret4 = IMU4.begin();
            if (ret4 != 0) {
                Serial.print("IMU4 ICM42670P I2C0initialization failed: ");
            }
            else {
                break;
            }
            vTaskDelay(50/ portTICK_PERIOD_MS);
        }
    }

#ifdef ACCEL
    // Accel ODR = 25 Hz and Full Scale Range = 16G
    IMU4.startAccel(GYRO_SR, 16);
#endif
    // Gyro ODR = 25 Hz and Full Scale Range = 2000 dps
    IMU4.startGyro(GYRO_SR, GYRO_FSR);
    // Wait IMU to start
    delay(100);
    ESP_LOGI(TAG, "IMU4 initialized...");
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

void initAds2() {
    delay(100);

    ads2.setGain(GAIN_ONE);
    ads2.setDataRate(RATE_ADS1115_32SPS);

    bool ret2;
    ret2 = ads2.begin(ADS1115_ADDRESS0, &Wire);

    if (ret2 != true) {
        ESP_LOGE(TAG, "Failed to initialize ADS2.");
        while(ret2 != true) {
            ret2 = ads2.begin(ADS1115_ADDRESS0, &Wire);
            if (ret2 != true) {
                ESP_LOGE(TAG, "Failed to initialize ADS2.");
            }
            else {
                break;
            }
            vTaskDelay(100/ portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "ADS2 initialized...");
}

void initAds3() {
    delay(100);

    ads3.setGain(GAIN_ONE);
    ads3.setDataRate(RATE_ADS1115_32SPS);

    bool ret3;
    ret3 = ads3.begin(ADS1115_ADDRESS1, &Wire1);

    if (ret3 != true) {
        ESP_LOGE(TAG, "Failed to initialize ADS3.");
        while(ret3 != true) {
            ret3 = ads3.begin(ADS1115_ADDRESS1, &Wire1);
            if (ret3 != true) {
                ESP_LOGE(TAG, "Failed to initialize ADS3.");
            }
            else {
                break;
            }
            vTaskDelay(100/ portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "ADS3 initialized...");
}

void initAds4() {
    delay(100);

    ads4.setGain(GAIN_ONE);
    ads4.setDataRate(RATE_ADS1115_32SPS);

    if (!ads4.begin(ADS1115_ADDRESS1, &Wire)) {
        ESP_LOGE(TAG, "Failed to initialize ADS4.");
    }

    bool ret4;
    ret4 = ads4.begin(ADS1115_ADDRESS1, &Wire);

    if (ret4 != true) {
        ESP_LOGE(TAG, "Failed to initialize ADS4.");
        while(ret4 != true) {
            ret4 = ads4.begin(ADS1115_ADDRESS1, &Wire);
            if (ret4 != true) {
                ESP_LOGE(TAG, "Failed to initialize ADS4.");
            }
            else {
                break;
            }
            vTaskDelay(100/ portTICK_PERIOD_MS);
        }
    }
    ESP_LOGI(TAG, "ADS4 initialized...");
}


void measureTof() {
    // Serial.print("Reading a measurement... ");
    // tofSensor.rangingTest(&tofMeasure, false); // pass in 'true' to get debug data printout!
    // tofSensor.getRangingMeasurement(&tofMeasure, false);
    // tofSensor.getSingleRangingMeasurement(&tofMeasure, false);
    // allData.M_tof = static_cast<int16_t>(tofMeasure.RangeMilliMeter);
    if (tofSensor.isRangeComplete())
    {
        allData.M_tof = static_cast<int16_t>(tofSensor.readRange());
    }
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
}

void measureImu2() {
    inv_imu_sensor_event_t imu_event2;

    imu_valid2 = IMU2.isGyroDataValid(&imu_event2);
    if (imu_valid2) {
        // Get last event
        IMU2.getDataFromRegisters(imu_event2);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU2.getDataFromRegisters(imu_event2);
    }

#ifndef BLE_TEST  
#ifdef ACCEL
    allData.M_accel2[0] = imu_event2.accel[0];
    allData.M_accel2[1] = imu_event2.accel[1];
    allData.M_accel2[2] = imu_event2.accel[2];
#endif
    allData.M_gyro2[0] = imu_event2.gyro[0];
    allData.M_gyro2[1] = imu_event2.gyro[1];
    allData.M_gyro2[2] = imu_event2.gyro[2];
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
}

void measureImu3() {
    inv_imu_sensor_event_t imu_event3;

    imu_valid3 = IMU3.isGyroDataValid(&imu_event3);
    if (imu_valid3) {
        // Get last event
        IMU3.getDataFromRegisters(imu_event3);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU3.getDataFromRegisters(imu_event3);
    }

#ifndef BLE_TEST  
#ifdef ACCEL
    allData.M_accel3[0] = imu_event3.accel[0];
    allData.M_accel3[1] = imu_event3.accel[1];
    allData.M_accel3[2] = imu_event3.accel[2];
#endif
    allData.M_gyro3[0] = imu_event3.gyro[0];
    allData.M_gyro3[1] = imu_event3.gyro[1];
    allData.M_gyro3[2] = imu_event3.gyro[2];
#else
    allData.M_accel3[0] = ble_test_int16t_value1;
    allData.M_accel3[1] = ble_test_int16t_value1;
    allData.M_accel3[2] = ble_test_int16t_value1;
    allData.M_gyro3[0] = ble_test_int16t_value1;
    allData.M_gyro3[1] = ble_test_int16t_value1;
    allData.M_gyro3[2] = ble_test_int16t_value1;
    ble_test_int16t_value1 += 256;
    if (ble_test_int16t_value1 > 65536) {
        ble_test_int16t_value1 = 0;
    }
#endif
}

void measureImu4() {

    inv_imu_sensor_event_t imu_event4;

    imu_valid4 = IMU4.isGyroDataValid(&imu_event4);
    if (imu_valid4) {
        // Get last event
        IMU4.getDataFromRegisters(imu_event4);
    }
    else {
        vTaskDelay(handler_delay / portTICK_PERIOD_MS);
        IMU4.getDataFromRegisters(imu_event4);
    }

#ifndef BLE_TEST  
#ifdef ACCEL
    allData.M_accel4[0] = imu_event4.accel[0];
    allData.M_accel4[1] = imu_event4.accel[1];
    allData.M_accel4[2] = imu_event4.accel[2];
#endif
    allData.M_gyro4[0] = imu_event4.gyro[0];
    allData.M_gyro4[1] = imu_event4.gyro[1];
    allData.M_gyro4[2] = imu_event4.gyro[2];
#else
    allData.M_accel4[0] = ble_test_int16t_value1;
    allData.M_accel4[1] = ble_test_int16t_value1;
    allData.M_accel4[2] = ble_test_int16t_value1;
    allData.M_gyro4[0] = ble_test_int16t_value1;
    allData.M_gyro4[1] = ble_test_int16t_value1;
    allData.M_gyro4[2] = ble_test_int16t_value1;
    ble_test_int16t_value1 += 256;
    if (ble_test_int16t_value1 > 65536) {
        ble_test_int16t_value1 = 0;
    }
#endif

}

void measureAds1() {
#ifndef BLE_TEST        
    allData.M_ads1 = ads1.continuous_readADC_SingleEnded(0);
#else
    allData.M_ads1 = ble_test_int16t_value2;
    ble_test_int16t_value2 += 256;
    if (ble_test_int16t_value2 > 65536) {
        ble_test_int16t_value2 = 0;
    }
#endif
}

void measureAds2() {
#ifndef BLE_TEST
    allData.M_ads2 = ads2.continuous_readADC_SingleEnded(0);
#else
    allData.M_ads1 = ble_test_int16t_value2;
    ble_test_int16t_value2 += 256;
    if (ble_test_int16t_value2 > 65536) {
        ble_test_int16t_value2 = 0;
    }
#endif      
}

void measureAds3() {
#ifndef BLE_TEST
    allData.M_ads3 = ads3.continuous_readADC_SingleEnded(0);
#else
    allData.M_ads1 = ble_test_int16t_value2;
    ble_test_int16t_value2 += 256;
    if (ble_test_int16t_value2 > 65536) {
        ble_test_int16t_value2 = 0;
    }
#endif    
}

void measureAds4() {
#ifndef BLE_TEST
    allData.M_ads4 = ads4.continuous_readADC_SingleEnded(0);
#else
    allData.M_ads1 = ble_test_int16t_value2;
    ble_test_int16t_value2 += 256;
    if (ble_test_int16t_value2 > 65536) {
        ble_test_int16t_value2 = 0;
    }
#endif    
}
