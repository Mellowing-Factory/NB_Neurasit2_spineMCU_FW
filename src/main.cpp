/* Developer Note
(cshin)
- Test Code: Code to test out new code (to be committed or deleted)
- Test Temp: Temporary code just for testing (comment or delete prior to commit)
- Test Silenced: Existing code that was silenced for testing (uncomment or delete prior to commit)

(cshin)
- Added sendWakeMsg() to send reset_reason to AWS server (20230918)
- Added previous_reset_reason to Wake Message (20230921)
*/

#include "globals.h"
#include <time.h>
#include <stdlib.h>
#include <Arduino.h>
#include <esp_coexist.h>
#include <Wire.h>
#include <SPI.h>

#include "timer.h"
#include "uartMaster.h"

#include "irqhandler.h"
#include "mallocator.h"
#include "bleData.h"
#include "resetCheck.h"
#include "bleServerHub.h"
#include <Preferences.h>
#include "allSensors.h"

static const char TAG[] = __FILE__;

TaskHandle_t irqHandlerTask = NULL;
TaskHandle_t TaskBle_h = NULL;
TaskHandle_t TaskData_h = NULL;
TaskHandle_t TasUartMaster_h = NULL;
TaskHandle_t TaskTimer_h = NULL;
TaskHandle_t TaskTimer2_h = NULL;
TaskHandle_t TaskSensors_h = NULL;

All_data_t allData; 
BleDataConvert bledata;

configData_t *cfg;

extern Preferences preferences;

void makeThingName() {
    byte mac[6];
    byte macBle[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    esp_read_mac(macBle, ESP_MAC_BT);
    sprintf(cfg->thing_name, "BS%02X%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], macBle[5]);
    Serial.printf("\n\ndevice id: %s\n", cfg->thing_name);
}

void initData() {
    cfg = ALLOCATION(configData_t, 1);

    dataBuffers = ALLOCATION(Data_message_t, ADS_BUFFER_NUM);
    adsBuffers = ALLOCATION(Ads_message_t, ADS_BUFFER_NUM);
    for (int loop = 0; loop < ADS_BUFFER_NUM; loop++) {
    	memset(&adsBuffers[loop], 0, sizeof(Ads_message_t));
    }

	ESP_LOGI(TAG, "Data initialized...");
}

void setup() {

	Serial.begin(115200);
  	delay(100);
	Serial.println("(Serial: ON)");
	
	ESP_LOGI(TAG, "\n\n-------------- Initializing --------------\n");
	initData();
	makeThingName();
	
	// bool dummy;
	// dummy = Wire.begin(static_cast<int>(SDA_PIN), static_cast<int>(SCL_PIN), static_cast<uint32_t>(400000));
	// Serial.println(dummy);	
	// Serial.print("MOSI: ");
	// Serial.println(MOSI);
	// Serial.print("MISO: ");
	// Serial.println(MISO);
	// Serial.print("SCK: ");
	// Serial.println(SCK);
	// Serial.print("SS: ");
	// Serial.println(SS);  
	bool dummy1;
	dummy1 = Wire1.begin(static_cast<int>(SDA2_PIN), static_cast<int>(SCL2_PIN), static_cast<uint32_t>(400000));
	// Serial.println(dummy1);

	byte error, address;
	int nDevices;
	Serial.println("Scanning...");
	nDevices = 0;
	for(address = 1; address < 127; address++ ) {
		Wire1.beginTransmission(address);
		error = Wire1.endTransmission();
		if (error == 0) {
		Serial.print("I2C device found at address 0x");
		if (address<16) {
			Serial.print("0");
		}
		Serial.println(address,HEX);
		nDevices++;
		}
		else if (error==4) {
		Serial.print("Unknow error at address 0x");
		if (address<16) {
			Serial.print("0");
		}
		Serial.println(address,HEX);
		}    
	}
	if (nDevices == 0) {
		Serial.println("No I2C devices found\n");
	}
	else {
		Serial.println("done\n");
	}
	

	delay(100);
	
	// initIRQTask();
    // delay(500);

	delay(100);
	initAllSensors();

    // delay(100);
	// initTimer2();

    delay(100);
	initTimer();

	delay(100);

    initBleServer();
	
	delay(100);

	vTaskDelete(NULL);
}

void loop() {
	
	vTaskDelete(NULL);
}