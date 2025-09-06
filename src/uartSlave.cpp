#include "uartSlave.h"

static const char TAG[] = __FILE__;

extern All_data_t allData;
bool sendData = false;

uint8_t message[MESSAGE_LENGTH];

void uartSlaveHandler(void *pvParameters) {
    while (1) {
        message[0] = 0;
        message[1] = 0;
        message[2] = 255;
        message[3] = 255;
        message[4] = 0;
        message[5] = 0;
    
        // Copy floats directly to message buffer
        memcpy(&message[6], &allData.roll0, sizeof(float));   // bytes 6-9
        memcpy(&message[10], &allData.roll1, sizeof(float));  // bytes 10-13
        memcpy(&message[14], &allData.roll2, sizeof(float));  // bytes 14-17
        memcpy(&message[18], &allData.roll3, sizeof(float));  // bytes 18-21

        if (sendData) {        
            Serial1.write(message, MESSAGE_LENGTH);
            printf("Pitch0: %.2f, Pitch1: %.2f, Pitch2: %.2f, Pitch3: %.2f\n", allData.roll0, allData.roll1, allData.roll2, allData.roll3);
            sendData = false;
        }
        vTaskDelay(75 / portTICK_RATE_MS);
    }
}

void initUartSlave() {

    Serial1.begin(115200, SERIAL_8N1, SLAVE_RX, SLAVE_TX);
  	delay(100);
	// Serial1.println("(Serial2: ON)");

    xTaskCreatePinnedToCore(uartSlaveHandler,
                            "uartSlaveHandler",
                            UART_TASK_STACK,
                            NULL,
                            UART_TASK_PRI,
                            &TaskUartSlave_h,
                            UART_TASK_CORE);
    ESP_LOGI(TAG, "ESP SLAVE initialized...");
}


// Receiver side code for reference
void parseReceivedMessage(uint8_t *receivedMessage, All_data_t *parsedData) {
    // Check header
    if (receivedMessage[0] == 0 && receivedMessage[1] == 0 && 
        receivedMessage[2] == 255 && receivedMessage[3] == 255 &&
        receivedMessage[4] == 0 && receivedMessage[5] == 0) {
        
        // Extract floats using memcpy
        memcpy(&parsedData->pitch0, &receivedMessage[6], sizeof(float));
        memcpy(&parsedData->pitch1, &receivedMessage[10], sizeof(float));
        memcpy(&parsedData->pitch2, &receivedMessage[14], sizeof(float));
        memcpy(&parsedData->pitch3, &receivedMessage[18], sizeof(float));
    }
}