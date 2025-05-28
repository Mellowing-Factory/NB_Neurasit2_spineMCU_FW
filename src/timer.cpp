#include "timer.h"
#include "tinyml.h"
#include "irqhandler.h"
#include "bleServerHub.h"
#include "bleData.h"
#include "globals.h"

static const char TAG[] = __FILE__;

uint8_t sendBufferId = 0;
uint8_t processBufferId = 0;

int16_t sensor_data_buffer[40];
uint8_t sensor_buffer_index = 0;

extern All_data_t allData;
extern BleDataConvert bledata;
extern PredictionDataClass predictionData;
extern bool startTransmission;
extern bool bleConnected;

/* Interrupt for setting sampling rate */
volatile int interruptCounter;
volatile int interrupt2Counter;
hw_timer_t * timer = NULL;
hw_timer_t * timer2 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t predictionBufferIndex = 0;
uint8_t predictionResult = 0;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
    // Serial.println("interrupt generated");
}

bool calculateSlope(int16_t data[], int16_t threshold, int duration) {
    bool prediction = false;
    int16_t slope = 0;
    for (int i = BUFFER_LEN-1-duration; i < BUFFER_LEN-1; i++) {
        slope += data[i+1]-data[i];
        Serial.printf("d_i+1 - d_i: %d\n", data[i+1]-data[i]);
    }
    slope /= duration;
    if (slope >= threshold) {
        prediction = true;
    }
    Serial.printf("slope: %d\n", slope);

    return prediction;
}

int dummyCounter = 0;
int dummyCounter2 = 0;
void timerHandler(void *pvParameters) {

    while (1) {
        if (interruptCounter > 0) {
            portENTER_CRITICAL(&timerMux);
            interruptCounter--;
            portEXIT_CRITICAL(&timerMux);
            if (bleConnected) {
                stream_data_to_phone(allData.M_ads1, allData.M_gyro1[0], allData.M_gyro1[1], allData.M_gyro1[2]);
                Serial.printf("%d, %d, %d, %d\n", allData.M_ads1, allData.M_gyro1[0], allData.M_gyro1[1], allData.M_gyro1[2]);
                // Serial.printf("%d, %d, %d, %d\n", allData.M_ads3, allData.M_gyro3[0], allData.M_gyro3[1], allData.M_gyro3[2]);
            }
        }
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

void initTimer() {
   
    // timer = timerBegin(1000000);
    // timerAttachInterrupt(timer, &onTimer);
    // timerAlarm(timer, 100000, true, 0);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 100000, true);
    timerAlarmEnable(timer);
  
    xTaskCreatePinnedToCore(timerHandler,
                            "timerHandler",
                            TIMER_TASK_STACK,
                            NULL,
                            TIMER_TASK_PRI,
                            &TaskTimer_h,
                            TIMER_TASK_CORE);
    ESP_LOGI(TAG, "Timer initialized...");
}