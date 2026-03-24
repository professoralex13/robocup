#include "scheduler_task.hpp"
#include "Arduino.h"

SchedulerTask::SchedulerTask(const char *task_name) : task_name(task_name) {}

void SchedulerTask::log(const char *format, ...) {
    Serial.printf("[LOG][%s]: ", task_name);

    va_list args;

    va_start(args, format);
    Serial.printf(format, args);
    va_end(args);

    Serial.println("");
}

void SchedulerTask::log_err(const char *format, ...) {
    Serial.printf("[ERR][%s]: ", task_name);

    va_list args;

    va_start(args, format);
    Serial.printf(format, args);
    va_end(args);

    Serial.println("");
}