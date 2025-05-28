#ifndef _BLE_SERVER_H
#define _BLE_SERVER_H

#include "globals.h"
#include "bleData.h"
#include "resetCheck.h"
#include <ArduinoJson.h>
#include <time.h>
#include <soc/rtc.h>
#include <NimBLEDevice.h>
#include "NimBLELog.h"
#include <stdio.h>
#include <Preferences.h>

/* ble characteristics */
#define SERVICE_UUID            "6e877c60-0e50-493f-b012-5a86acf4610e"
#define CHARACTERISTIC_UUID_RX  "6e877c61-0e50-493f-b012-5a86acf4610e"
#define CHARACTERISTIC_UUID_TX  "6e877c62-0e50-493f-b012-5a86acf4610e"

/* ble responses */
#define DEVICE_SUCCESS 0xFD
#define DEVICE_NOT_MATCHING 0xFF
#define DEVICE_BUSY 0xFE
#define DEVICE_NOT_INITIAL 0xFC
#define DEVICE_FAILED_CONN_WIFI 0x10

/* ble command */
#define HEAD_INIT_HUB_CONNECTING 0x18       // 24
#define HEAD_REQUEST_SAVED_DATA 0x17        // 23
#define HEAD_INIT_HUB_RESET 0x15            // 21   factory reset

void TaskBle(void *pvParameters);
void deleteTaskBle();
void initBleServer();

void stream_data_to_phone(int16_t val0, int16_t val1, int16_t val2, int16_t val3);
void send_saved_data_to_phone(uint8_t predResult, int16_t TOF_buffer[]);
void send_saved_data_to_phone2(int16_t data_buffer[]);


void send_thing_name_to_phone();
void send_confirm_to_phone(byte flag, byte data);

#ifdef __cplusplus
extern "C" {
#endif

void createBLEServer();

#ifdef __cplusplus
}
#endif


#endif