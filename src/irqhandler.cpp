
#include "irqhandler.h"
#include "resetCheck.h"
#include <Preferences.h>

extern Preferences preferences;
static const char TAG[] = __FILE__;
static bool irqMask = false;

// irq handler task, handles all our application level interrupts
void irqHandler(void *pvParameters) {
    configASSERT(((uint32_t)pvParameters) == 1); // FreeRTOS check
    uint32_t InterruptStatus;

    // task remains in blocked state until it is notified by an irq
    for (;;) {
        xTaskNotifyWait(0x00,             // Don't clear any bits on entry
                        ULONG_MAX,        // Clear all bits on exit
                        &InterruptStatus, // Receives the notification value
                        portMAX_DELAY);   // wait forever
        if (irqMask) {
            continue;
        }

        if (InterruptStatus & SENDCYCLE_IRQ) {
            // sendSensorData();
        }
        if (InterruptStatus & RESET_MANU) {
            deviceResetManu();
        }
    }
}

void initIRQTask() {
    xTaskCreatePinnedToCore(irqHandler,      // task function
                            "irqhandler",    // name of task
                            IRQ_TASK_STACK,  // stack size of task
                            (void *)1,       // parameter of the task
                            IRQ_TASK_PRI,    // priority of the task
                            &TaskIrq, // task handle
                            IRQ_TASK_CORE);  // CPU core
    ESP_LOGI(TAG, "IRQ Handler initialized...");
}

void notifyIrq(uint32_t uvalue) {
    if (TaskIrq == NULL) {
        return;
    }
    xTaskNotifyFromISR(TaskIrq, uvalue, eSetBits, NULL);
}

void deleteIRQTask() {
    if (TaskIrq != NULL) {
        vTaskDelete(TaskIrq);
    }
}

void setIrqMask(bool set) {
    irqMask = set;
}
