#include "drive_train.hpp"
#include "imu.hpp"
#include "lidar.hpp"
#include <Arduino.h>
#include <memory>

static LidarTask lidar_task = LidarTask(Serial1);
static DriveTrainTask drive_train_task = DriveTrainTask(0, 1);
static ImuTask imu_task = ImuTask(&Wire);

const size_t NUM_TASKS = 3;

std::array<SchedulerTask *, NUM_TASKS> tasks = {&lidar_task, &drive_train_task, &imu_task};
std::array<uint32_t, NUM_TASKS> next_runs = {0};

void setup() {
    Serial.begin(9600);

    for (size_t i = 0; i < NUM_TASKS; i++) {
        tasks[i]->setup();
    }
}

void loop() {
    uint32_t ticks = micros();

    for (size_t i = 0; i < NUM_TASKS; i++) {
        if (ticks >= next_runs[i]) {
            tasks[i]->loop();

            next_runs[i] += tasks[i]->get_period_micros();
        }
    }
}
