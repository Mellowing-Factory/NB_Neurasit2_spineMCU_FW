#include "uartMaster.h"

static const char TAG[] = __FILE__;

extern All_data_t allData;

uint8_t byte1 = 0x01;
uint8_t byte2 = 0x02;
int16_t value = ((int16_t)byte2 << 8) | byte1;
uint8_t incomingByte = 255;
uint8_t incomingByteArray[SLAVE_MESSAGE_LENGTH*2];
uint8_t byteValue = 0;

bool dataAvailable = false;
int byteIndex = 0;
int startIndex = 0;
int messageLength = SLAVE_MESSAGE_LENGTH-6;
int byte_counter = 0;
int sensor_counter = 0;

extern bool startTransmission;

#ifdef BLE_TEST
    int16_t ble_test_int16t_value = 0;
#endif

void uartMasterHandler(void *pvParameters) {
    while (1) {

        // Serial.println(Serial1.available());
        // Serial.print(Serial1.readString());      
        
        while (Serial1.available()) {
            // Serial.print(Serial1.readString());
            // Serial.println(Serial1.read());

            incomingByte = Serial1.read();
            incomingByteArray[byteIndex] = incomingByte;
            byteIndex += 1;
        }
        if (byteIndex > 0) { dataAvailable = true; }
        byteIndex = 0;
        // Serial.println(dataAvailable);

#ifdef BLE_TEST
        dataAvailable = true;
#endif

        if (dataAvailable) {
#ifndef BLE_TEST            
            for (int i=0; i < SLAVE_MESSAGE_LENGTH*2; i++) {
                if (incomingByteArray[i] == 0 && incomingByteArray[i+1] == 0 && incomingByteArray[i+2] == 255 && incomingByteArray[i+3] == 255 && incomingByteArray[i+4] == 0 && incomingByteArray[i+5] == 0) {
                    startIndex = i + 6;
                }
            }
            // for (int i=startIndex; i < startIndex+14; i++) {
            //     Serial.println(incomingByteArray[i]);
            // }
            byte_counter = 0;
            sensor_counter = 0;
#else
            startIndex = 0;
            ble_test_int16t_value += 256;
            if (ble_test_int16t_value > 65536) {
                ble_test_int16t_value = 0;
            }
#endif
            for (int i=startIndex; i < startIndex+messageLength; i++) {
                byteValue = incomingByteArray[i];
                // Serial.println(incomingByteArray[i]);
                if (byte_counter == 0) {
                    byte1 = byteValue;
                    byte_counter += 1;
                }
                else if (byte_counter == 1) {
                    byte2 = byteValue;
                    byte_counter = 0;

#ifndef BLE_TEST
                    value = ((byte1 << 8) + byte2);
                    // value = 0;
#else
                    value = ble_test_int16t_value;
#endif
                    // Serial.print("             ");
                    // Serial.println(value);

#ifdef ACCEL
                    if (sensor_counter == 0 ) {
                        allData.S_accel0[0] = value;
                    }
                    else if (sensor_counter == 1) {
                        allData.S_accel0[1] = value;
                    }
                    else if (sensor_counter == 2) {
                        allData.S_accel0[2] = value;
                    }
                    else if (sensor_counter == 3) {
                        allData.S_gyro0[0] = value;
                    }
                    else if (sensor_counter == 4) {
                        allData.S_gyro0[1] = value;
                    }
                    else if (sensor_counter == 5) {
                        allData.S_gyro0[2] = value;
                    }

                    else if (sensor_counter == 6) {
                        allData.S_ads1 = value;
                    }
                    else if (sensor_counter == 7) {
                        allData.S_accel1[0] = value;
                    }
                    else if (sensor_counter == 8) {
                        allData.S_accel1[1] = value;
                    }
                    else if (sensor_counter == 9) {
                        allData.S_accel1[2] = value;
                    }
                    else if (sensor_counter == 10) {
                        allData.S_gyro1[0] = value;
                    }
                    else if (sensor_counter == 11) {
                        allData.S_gyro1[1] = value;
                    }
                    else if (sensor_counter == 12) {
                        allData.S_gyro1[2] = value;
                    }

                    else if (sensor_counter == 13) {
                        allData.S_ads2 = value;
                    }
                    else if (sensor_counter == 14) {
                        allData.S_accel2[0] = value;
                    }
                    else if (sensor_counter == 15) {
                        allData.S_accel2[1] = value;
                    }
                    else if (sensor_counter == 16) {
                        allData.S_accel2[2] = value;
                    }
                    else if (sensor_counter == 17) {
                        allData.S_gyro2[0] = value;
                    }
                    else if (sensor_counter == 18) {
                        allData.S_gyro2[1] = value;
                    }
                    else if (sensor_counter == 19) {
                        allData.S_gyro2[2] = value;
                    }

                    else if (sensor_counter == 20) {
                        allData.S_ads3_1 = value;
                    }
                    // else if (sensor_counter == 21) {
                    //     allData.S_ads3_2 = value;
                    // }
                    else if (sensor_counter == 21) {
                        allData.S_ads3_3 = value;
                    }
                    // else if (sensor_counter == 23) {
                    //     allData.S_ads3_4 = value;
                    // }
#else
                    if (sensor_counter == 0) {
                        allData.S_gyro0[0] = value;
                    }
                    else if (sensor_counter == 1) {
                        allData.S_gyro0[1] = value;
                    }
                    else if (sensor_counter == 2) {
                        allData.S_gyro0[2] = value;
                    }

                    else if (sensor_counter == 3) {
                        allData.S_ads1 = value;
                    }
                    else if (sensor_counter == 4) {
                        allData.S_gyro1[0] = value;
                    }
                    else if (sensor_counter == 5) {
                        allData.S_gyro1[1] = value;
                    }
                    else if (sensor_counter == 6) {
                        allData.S_gyro1[2] = value;
                    }

                    else if (sensor_counter == 7) {
                        allData.S_ads2 = value;
                    }
                    else if (sensor_counter == 8) {
                        allData.S_gyro2[0] = value;
                    }
                    else if (sensor_counter == 9) {
                        allData.S_gyro2[1] = value;
                    }
                    else if (sensor_counter == 10) {
                        allData.S_gyro2[2] = value;
                    }

                    else if (sensor_counter == 11) {
                        allData.S_ads3_1 = value;
                    }
                    else if (sensor_counter == 12) {
                        allData.S_ads3_3 = value;
                    }
#endif
                    
                    sensor_counter += 1;
                    if (sensor_counter == NUM_SLAVE_SENSORS) { 
                        sensor_counter = 0;
                    }
                }
            }
            // Serial.print("S IMU0: "); Serial.print(allData.S_accel0[0]); Serial.print(", "); Serial.print(allData.S_accel0[1]); Serial.print(", "); Serial.print(allData.S_accel0[2]); Serial.print(", "); Serial.print(allData.S_gyro0[0]); Serial.print(", "); Serial.print(allData.S_gyro0[1]); Serial.print(", "); Serial.println(allData.S_gyro0[2]);
            // Serial.print("S ADS1: "); Serial.println(allData.S_ads1);
            // Serial.print("S IMU1: "); Serial.print(allData.S_accel1[0]); Serial.print(", "); Serial.print(allData.S_accel1[1]); Serial.print(", "); Serial.print(allData.S_accel1[2]); Serial.print(", "); Serial.print(allData.S_gyro1[0]); Serial.print(", "); Serial.print(allData.S_gyro1[1]); Serial.print(", "); Serial.println(allData.S_gyro1[2]);
            // Serial.print("S ADS2: "); Serial.println(allData.S_ads2);
            // Serial.print("S IMU2: "); Serial.print(allData.S_accel2[0]); Serial.print(", "); Serial.print(allData.S_accel2[1]); Serial.print(", "); Serial.print(allData.S_accel2[2]); Serial.print(", "); Serial.print(allData.S_gyro2[0]); Serial.print(", "); Serial.print(allData.S_gyro2[1]); Serial.print(", "); Serial.println(allData.S_gyro2[2]);
            // Serial.println(" ");
            
        }
        byteIndex = 0;
        dataAvailable = false;
        // Serial.println(dataAvailable);
        vTaskDelay(50 /  portTICK_RATE_MS);
    }
}

bool initUartMaster() {

    Serial1.begin(115200, SERIAL_8N1, SLAVE_RX, SLAVE_TX);
  	delay(100);
	Serial1.println("(Serial2: ON)");

    xTaskCreatePinnedToCore(uartMasterHandler,
                            "uartMasterHandler",
                            UART_TASK_STACK,
                            NULL,
                            UART_TASK_PRI,
                            &TasUartMaster_h,
                            UART_TASK_CORE);
    ESP_LOGI(TAG, "ESP MASTER initialized...");
}