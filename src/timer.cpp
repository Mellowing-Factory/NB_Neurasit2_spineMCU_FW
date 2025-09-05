#include "timer.h"

static const char TAG[] = __FILE__;

/* Interrupt for setting sampling rate */
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int interruptCounter;

extern bool sendData;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void timerHandler(void *pvParameters) {

    while (1) {
        if (interruptCounter > 0) {
            portENTER_CRITICAL(&timerMux);
            interruptCounter--;
            portEXIT_CRITICAL(&timerMux);
            sendData = true;
        }
        // Serial.println("0.1 seconds passed\n");
        vTaskDelay(5 / portTICK_PERIOD_MS);
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