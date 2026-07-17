#include "tasks/telemetry.hpp"
#include "Arduino.h"
#include "telemetry_bus.hpp"
#include <wiring.h>

TelemetryTask::TelemetryTask() : SchedulerTask("telemetry_task") {}

void TelemetryTask::setup() {
    Serial.begin(921600);
    telemetry::begin(&Serial);
}

void TelemetryTask::loop() { telemetry::flush_values(); }
