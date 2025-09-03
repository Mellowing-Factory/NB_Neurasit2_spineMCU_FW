#include "globals.h"
#include <time.h>
#include <stdlib.h>
#include <Arduino.h>
#include <esp_coexist.h>
#include <Wire.h>
#include "timer.h"
#include "irqhandler.h"
#include "mallocator.h"
#include "resetCheck.h"
#include "allSensors.h"
#include "customAlgorithms.h"
#include "led.h"

static const char TAG[] = __FILE__;

TaskHandle_t TaskIrq = NULL;
TaskHandle_t TaskData_h = NULL;
TaskHandle_t TaskTimer_h = NULL;
TaskHandle_t TaskSensors_h = NULL;
TaskHandle_t TaskUartSlave_h = NULL;
TaskHandle_t TaskUartMaster_h = NULL;

All_data_t allData; 
Kalman_t kalman;


void setup() {

	Serial.begin(115200);
  	delay(100);
	Serial.println("(Serial: ON)");
	
	ESP_LOGI(TAG, "\n\n-------------- Initializing --------------\n");
	
	bool dummy0;
	dummy0 = Wire.begin(static_cast<int>(SDA_PIN), static_cast<int>(SCL_PIN), static_cast<uint32_t>(400000));
	Serial.println(dummy0);
	bool dummy1;
	dummy1 = Wire1.begin(static_cast<int>(SDA2_PIN), static_cast<int>(SCL2_PIN), static_cast<uint32_t>(400000));
	Serial.println(dummy1);

	// byte error, address;
	// int nDevices;
	// Serial.println("Scanning...");
	// nDevices = 0;
	// for(address = 1; address < 127; address++ ) {
	// 	Wire1.beginTransmission(address);
	// 	error = Wire1.endTransmission();
	// 	if (error == 0) {
	// 	Serial.print("I2C device found at address 0x");
	// 	if (address<16) {
	// 		Serial.print("0");
	// 	}
	// 	Serial.println(address,HEX);
	// 	nDevices++;
	// 	}
	// 	else if (error==4) {
	// 	Serial.print("Unknow error at address 0x");
	// 	if (address<16) {
	// 		Serial.print("0");
	// 	}
	// 	Serial.println(address,HEX);
	// 	}    
	// }
	// if (nDevices == 0) {
	// 	Serial.println("No I2C devices found\n");
	// }
	// else {
	// 	Serial.println("done\n");
	// }
	

	delay(100);
	initAllSensors();

    delay(100);
	initTimer();

	initKalman(&kalman);

	initLed();
	turnLedOn(LED_PIN_GR);

	vTaskDelete(NULL);
}

void loop() {
	
	vTaskDelete(NULL);
}