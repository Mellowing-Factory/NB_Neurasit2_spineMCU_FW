
#include "resetCheck.h"
#include <rom/rtc.h>

static const char TAG[] = __FILE__;

JsonObject reset_json;

void restartApp() {
	ESP.restart();
}