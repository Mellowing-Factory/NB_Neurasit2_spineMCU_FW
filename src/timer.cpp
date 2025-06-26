#include "timer.h"


static const char TAG[] = __FILE__;

// extern PredictionDataClass predictionData;
extern bool startTransmission;
extern bool bleConnected;
extern uint8_t breathingRate;
extern uint8_t breathingRateSmoothed;
extern uint8_t tilt_count;
extern uint8_t walkTimeSec;
extern uint8_t weight;
extern bool isMoving;

/* Interrupt for setting sampling rate */
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux3 = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;
volatile int interruptCounter2;
volatile int interruptCounter3;

hw_timer_t * timer2 = NULL;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;
volatile int sixtySecondsCounter;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;     // NOTHING
    portEXIT_CRITICAL_ISR(&timerMux);
    portENTER_CRITICAL_ISR(&timerMux2);
    interruptCounter2++;     // IMU MEASUREMENTS
    portEXIT_CRITICAL_ISR(&timerMux2);
    portENTER_CRITICAL_ISR(&timerMux3);
    interruptCounter3++;     // ADS MEASUREMENTS
    portEXIT_CRITICAL_ISR(&timerMux3);
    // Serial.println("100ms passed");
}
void IRAM_ATTR onTimer2() {
    portENTER_CRITICAL_ISR(&timer2Mux);
    sixtySecondsCounter++;     // DATA SEND TIMER
    portEXIT_CRITICAL_ISR(&timer2Mux);
}

void timerHandler(void *pvParameters) {

    while (1) {
        // if (interruptCounter > 0) {
        //     portENTER_CRITICAL(&timerMux);
        //     interruptCounter--;
        //     portEXIT_CRITICAL(&timerMux);
        //     if (bleConnected) {
        //         stream_data_to_phone(allData.M_ads1, allData.M_gyro1[0], allData.M_gyro1[1], allData.M_gyro1[2]);
        //         Serial.printf("%d, %d, %d, %d\n", allData.M_ads1, allData.M_gyro1[0], allData.M_gyro1[1], allData.M_gyro1[2]);
        //         // Serial.printf("%d, %d, %d, %d\n", allData.M_ads3, allData.M_gyro3[0], allData.M_gyro3[1], allData.M_gyro3[2]);
        //     }
        // }
        if (sixtySecondsCounter > 0) {
            portENTER_CRITICAL(&timer2Mux);
            sixtySecondsCounter--;
            portEXIT_CRITICAL(&timer2Mux);
            if (bleConnected) {
                float energyExpenditure = 0.2*tilt_count + walkTimeSec*3.0f*weight/3600.0f +1.0f;
                if (breathingRateSmoothed >= 17 && breathingRateSmoothed < 20) {
                    energyExpenditure = energyExpenditure * 1.2;
                }
                else if (breathingRateSmoothed >= 20 && breathingRateSmoothed < 23) {
                    energyExpenditure = energyExpenditure * 1.5;
                }
                else if (breathingRateSmoothed >= 23) {
                    energyExpenditure = energyExpenditure * 1.8;
                }
                
                uint8_t energyKcal = (uint8_t)(energyExpenditure + 0.5f); // round to nearest integer
                send_LF_BR_EE_ble(tilt_count, breathingRateSmoothed, energyKcal);
                Serial.printf("Walk time: %u, tilt count: %u, breathing rate: %u, energy expenditure: %u\n", walkTimeSec, tilt_count, breathingRateSmoothed, energyKcal);
                tilt_count = 0;      
                walkTimeSec = 0;       
            }
            Serial.println("60 seconds passed\n");
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
    timer2 = timerBegin(1, 80, true);
    timerAttachInterrupt(timer2, &onTimer2, true);
    timerAlarmWrite(timer2, 60000000, true);
    timerAlarmEnable(timer2);

    xTaskCreatePinnedToCore(timerHandler,
                            "timerHandler",
                            TIMER_TASK_STACK,
                            NULL,
                            TIMER_TASK_PRI,
                            &TaskTimer_h,
                            TIMER_TASK_CORE);
    ESP_LOGI(TAG, "Timer initialized...");
}