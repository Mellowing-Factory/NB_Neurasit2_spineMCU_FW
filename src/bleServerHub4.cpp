#include "bleServerHub.h"

static NimBLEServer* pServer;
NimBLECharacteristic* pTxCharacteristic;
static uint16_t clientID;

bool startTransmission = false;
bool isRestartCheck = false;
unsigned long bleserverActivatedTime = 0;
bool bleServerIsOn = false;

struct tm newTimeInfo;  // To store the time received

String dummyString = "dummyString";
String userID = "";

bool bleConnected = false;

uint8_t weight = 70;

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
        printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 3x interval time for best results.
         */
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 18);
        bleserverActivatedTime = millis();
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
        // printf("Client disconnected - start advertising\n");
        // NimBLEDevice::startAdvertising();
        printf("Client disconnected: %s, reason: %d \n", connInfo.getAddress().toString().c_str(), reason);
        printf("heap : %d\n", ESP.getFreeHeap());
        deleteTaskBle();
    };

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) {
        printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 60);
    };
};

/** Handler class for characteristic actions */
class DataReceivedCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
        printf("%s : onRead(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
        bleserverActivatedTime = millis();
        // printf("%s : onWrite(), value: %s\n",
        //        pCharacteristic->getUUID().toString().c_str(),
        //        pCharacteristic->getValue().c_str());

        std::string value = pCharacteristic->getValue();
        printf("\nBLE command received! received value:\n");
        printf(value.c_str());
        Serial.println(value.length());

        if (value.length() > 0) {
            printf("BLE command %d___\n", value[0]);
            switch (value[0]) {
                case HEAD_INIT_HUB_CONNECTING: {
                    printf("BLE: HEAD_INIT_HUB_CONNECTING\n");
                    send_thing_name_to_phone();
                    getWeight(value[1]);
                    send_confirm_to_phone(value[0], DEVICE_SUCCESS);
                    bleConnected = true;
                    break;
                }
                case HEAD_REQUEST_SAVED_DATA: {
                    printf("BLE: HEAD_REQUEST_SAVED_DATA\n");
                    // send_saved_data_to_phone(dummyString);
                    send_confirm_to_phone(value[0], DEVICE_SUCCESS);
                    break;
                }
                case HEAD_INIT_HUB_RESET: {
                    isRestartCheck = true;
                    printf("BLE: HEAD_INIT_HUB_RESET\n");
                    send_confirm_to_phone(value[0], DEVICE_SUCCESS);
                    break;
                }

                default: {
                    printf("Invalid massage %d\n", value[0]);
                    send_confirm_to_phone(value[0], DEVICE_NOT_MATCHING);
                    break;
                }
            }
        }
    }
};

class DataWriteCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
        printf("%s : onWrite(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
        printf("%s : onRead(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }
};

// static CharacteristicCallbacks chrCallbacks;
static DataReceivedCallbacks datReceiCallbacks;
static DataWriteCallbacks    datWritCallbacks;

void createBLEServer(void) {
    printf("Starting NimBLE Server\n");

    /** sets device name */
    NimBLEDevice::init(BLE_ADVERTISE_NAME);
    NimBLEDevice::setMTU(512);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pBaadService = pServer->createService(SERVICE_UUID);
    printf("Created BLE service\n");

    pTxCharacteristic = pBaadService->createCharacteristic(
                                                CHARACTERISTIC_UUID_TX,
                                                NIMBLE_PROPERTY::NOTIFY
                                                );
    pTxCharacteristic->setValue("TX");
    pTxCharacteristic->setCallbacks(&datWritCallbacks);

    NimBLECharacteristic* pRxCharacteristic = pBaadService->createCharacteristic(
                                                CHARACTERISTIC_UUID_RX,
                                                NIMBLE_PROPERTY::WRITE |
                                                NIMBLE_PROPERTY::WRITE_NR
                                                );
    pRxCharacteristic->setValue("RX"); 
    pRxCharacteristic->setCallbacks(&datReceiCallbacks);
    printf("Created BLE characteristics \n");

    /** Start the services when finished creating all Characteristics and Descriptors */
    pBaadService->start();
    printf("1\n");

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    printf("2\n");
    /** Add the services to the advertisment data **/
    pAdvertising->addServiceUUID(pBaadService->getUUID());
    printf("2\n");
    pAdvertising->setManufacturerData(cfg->thing_name);
    // printf("2\n");
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    // pAdvertising->setScanResponse(false);
    pAdvertising->start();

    printf("Advertising Started\n");

}

void initBleServer() {
    bleServerIsOn = true;
    createBLEServer();
    printf("After Ble - Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
    xTaskCreatePinnedToCore(TaskBle, 
                            "TaskBle", 
                            BLE_TASK_STACK, 
                            NULL, 
                            BLE_TASK_PRI, 
                            &TaskBle_h, 
                            BLE_TASK_CORE);
}

void TaskBle(void *pvParameters) {
    (void)pvParameters;
    bleserverActivatedTime = millis();
    while (1) {
        // delete BLE task if user has not touched it for more than 30seconds
        // if (millis() - bleserverActivatedTime > 30000) {
        //     printf("deleting ble task due to no command from user\n");
        //     deleteTaskBle();
        //     printf("After Stopping Ble - Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
        // }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void deleteTaskBle() {
    restartApp(rst_ble_server_timeout);
    if (TaskBle_h != NULL) {
        vTaskDelete(TaskBle_h);
        TaskBle_h = NULL;
    }
    bleServerIsOn = false;

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) {
        pAdvertising->stop();
        printf("BLE Advertising stopped.\n");
    }

    // De-initialize BLE to release all resources
    NimBLEDevice::deinit();
    printf("BLE de-initialized.\n");
    // restartApp(rst_ble_server_timeout);
    // if (TaskBle_h != NULL) {
    //     vTaskDelete(TaskBle_h);
    //     TaskBle_h = NULL;
    // }
    // printf("After Stopping Ble - Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
}

// void send_saved_data_to_phone(uint8_t predResult, int16_t TOF_buffer[], int16_t ADS1_buffer[], int16_t IMU1_x_buffer[], int16_t IMU1_y_buffer[], int16_t IMU1_z_buffer[]) {
//     bledata.reset();
//     bledata.addByte(0xFF); 
//     bledata.addByte(predResult);
//     bledata.addByte(0xFF); // idx=2
//     bledata.addInt16Array(TOF_buffer, BUFFER_LEN); // idx=[3:63]
//     bledata.addByte(0xFF); // idx=63
//     bledata.addInt16Array(ADS1_buffer, BUFFER_LEN); // idx=[64:124]
//     bledata.addByte(0xFF); // idx=124
//     bledata.addInt16Array(IMU1_x_buffer, BUFFER_LEN); // idx=[125:185]
//     bledata.addByte(0xFF); // idx=185
//     bledata.addInt16Array(IMU1_y_buffer, BUFFER_LEN); // idx=[186:246]
//     bledata.addByte(0xFF); // idx=246
//     bledata.addInt16Array(IMU1_z_buffer, BUFFER_LEN); // idx=[247:307]
//     pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
//     pTxCharacteristic->notify();
//     // printf("Sent saved data of size %d to phone\n", bledata.getSize());
// }

void stream_data_to_phone(int16_t val0, int16_t val1, int16_t val2, int16_t val3) {
    bledata.reset();
    bledata.addByte(0xFF); 
    bledata.addInt16(val0);
    bledata.addInt16(val1);
    bledata.addInt16(val2);
    bledata.addInt16(val3);
    pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
    pTxCharacteristic->notify();
    // printf("Sent saved data of size %d to phone\n", bledata.getSize());
}

void send_LF_BR_EE_ble(uint8_t lumbarFlexion, uint8_t breathingRate, uint8_t energyExpenditure) {
    bledata.reset();
    bledata.addByte(0xFF); 
    bledata.addByte(0xFF); 
    bledata.addByte(0xFF); 
    bledata.addByte(lumbarFlexion);
    bledata.addByte(breathingRate);
    bledata.addByte(energyExpenditure);
    pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
    pTxCharacteristic->notify();
}

void send_saved_data_to_phone(uint8_t predResult, int16_t TOF_buffer[]) {
    bledata.reset();
    bledata.addByte(0xFF); 
    bledata.addByte(predResult);
    bledata.addByte(0xFF); // idx=2
    bledata.addInt16Array(TOF_buffer, BUFFER_LEN); // idx=[3:63]
    pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
    pTxCharacteristic->notify();
    // printf("Sent saved data of size %d to phone\n", bledata.getSize());
}

void send_saved_data_to_phone2(int16_t data_buffer[]) {
    bledata.reset();
    bledata.addByte(0xFF); 
    bledata.addByte(0xFF);
    bledata.addByte(0xFF); // idx=2
    bledata.addInt16Array(data_buffer, BUFFER_LEN); // idx=[3:63]
    pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
    pTxCharacteristic->notify();
    // printf("Sent saved data of size %d to phone\n", bledata.getSize());
}

void send_thing_name_to_phone() {
    bledata.reset();
    bledata.addthingname(cfg->thing_name, strlen(cfg->thing_name));
    pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
    pTxCharacteristic->notify();
    printf("Sent thing name to phone: %s\n Size: %d\n", cfg->thing_name, bledata.getSize());
}

void send_confirm_to_phone(byte flag, byte data) {
    bledata.reset();
    bledata.addByte(flag);
    bledata.addByte(data);
    pTxCharacteristic->setValue(bledata.getBuffer(), bledata.getSize());
    pTxCharacteristic->notify();
}

void getWeight(byte data) {
    uint8_t dummy = (uint8_t)data;
    if (dummy > 30 && dummy < 130) {
        weight = dummy;
    }
    printf("new weight is: %u\n", weight);
}