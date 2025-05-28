/* Developer Note
(cshin)
- Test Code: Code to test out new code (to be committed or deleted)
- Test Temp: Temporary code just for testing (comment or delete prior to commit)
- Test Silenced: Existing code that was silenced for testing (uncomment or delete prior to commit)

(cshin)
- combined manual reset with reset function (20230918)
- Fix to prev_reset_reason, for intentional resets update previous reason (20230925)
//*/

#include "resetCheck.h"
#include <rom/rtc.h>

static const char TAG[] = __FILE__;

JsonObject reset_json;

void restartApp(reset_reason_t reason) {
    vTaskDelay(1500 / portTICK_PERIOD_MS);

	ESP_LOGE(TAG, "restart_app reason: [%d]", reason);
	ESP.restart();
}

void deviceResetManu() {
    restartApp(rst_device_reset_manu);
}