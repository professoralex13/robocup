#include "lidar.hpp"
#include <Arduino.h>
#include <memory>

static LidarTask lidar_task = LidarTask(Serial1);

const size_t NUM_TASKS = 1;

std::array<SchedulerTask *, NUM_TASKS> tasks = {&lidar_task};
std::array<uint32_t, NUM_TASKS> next_runs = {0};

void setup() {
    for (size_t i = 0; i < NUM_TASKS; i++) {
        tasks[i]->setup();
    }
}

void loop() {
    double ticks = micros();

    for (size_t i = 0; i < NUM_TASKS; i++) {
        if (ticks >= next_runs[i]) {
            tasks[i]->loop();

            next_runs[i] += tasks[i]->get_period_micros();
        }
    }
}
