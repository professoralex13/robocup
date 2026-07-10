#include "intake_task.hpp"
#include "lidar.hpp"
#include "telemetry.hpp"
#include "user_command.hpp"
#include <Arduino.h>
#include <memory>

static LidarTask lidar_task = LidarTask();
static DriveTrainTask drive_train_task = DriveTrainTask();
static ImuTask imu_task = ImuTask(&Wire);
static PositionTrackingTask position_tracking_task =
    PositionTrackingTask(&imu_task, &drive_train_task);
static IntakeTask intake_task = IntakeTask();
// static TelemetryTask telemetry_task =
//     TelemetryTask(&lidar_task, &imu_task, &drive_train_task, &position_tracking_task);
// static UserCommandTask user_command_task = UserCommandTask(&drive_train_task);

const size_t NUM_TASKS = 5;

std::array<SchedulerTask *, NUM_TASKS> tasks = {
    &lidar_task, &drive_train_task, &imu_task, &position_tracking_task, &intake_task,
};
std::array<uint32_t, NUM_TASKS> next_runs = {0};

struct TimingData {
    int32_t total_samples;
    int32_t average_time;
    int32_t min_time;
    int32_t max_time;
};

std::array<TimingData, NUM_TASKS> task_timing_data = {{{
    0,
    0,
    100000,
    0,
}}};

#define LOG_TASK_STATS true

void setup() {
    Serial.begin(9600);

    for (size_t i = 0; i < NUM_TASKS; i++) {
        tasks[i]->setup();
    }
}

static uint32_t next_log_timings = 0;

void loop() {
    // Loop through tasks, run them, and log the time taken
    for (size_t i = 0; i < NUM_TASKS; i++) {
        uint32_t ticks = micros();

        if (ticks >= next_runs[i]) {
            tasks[i]->loop();
            int32_t time_taken = micros() - ticks;

            task_timing_data[i].total_samples++;
            task_timing_data[i].average_time +=
                (time_taken - task_timing_data[i].average_time) / task_timing_data[i].total_samples;

            if (task_timing_data[i].max_time < time_taken) {
                task_timing_data[i].max_time = time_taken;
            }

            if (task_timing_data[i].min_time > time_taken) {
                task_timing_data[i].min_time = time_taken;
            }

            int total_period = tasks[i]->get_period_micros();

            next_runs[i] += total_period;
        }
    }

    // Handle logging of task statistics
    if (micros() >= next_log_timings) {
        next_log_timings += 5e6;

        if (!LOG_TASK_STATS) {
            return;
        }

        Serial.println("=======BEGIN TIMING INFO=======");

        int summed_average_time = 0;
        int summed_period_time = 0;
        for (size_t i = 0; i < NUM_TASKS; i++) {
            Serial.printf("%s: %dus\n", tasks[i]->task_name, tasks[i]->get_period_micros());

            Serial.printf("|  Average: %dus\n", task_timing_data[i].average_time);
            Serial.printf("|  Min: %dus\n", task_timing_data[i].min_time);
            Serial.printf("|  Max: %dus\n", task_timing_data[i].max_time);
            Serial.printf("|  Average task load: %d%%\n",
                          (int)(100.0 * (float)task_timing_data[i].average_time /
                                (float)tasks[i]->get_period_micros()));

            summed_average_time += task_timing_data[i].average_time;
            summed_period_time += tasks[i]->get_period_micros();
        }

        Serial.printf("Average CPU load: %d%%\n",
                      (int)(100.0 * (float)summed_average_time / (float)summed_period_time));

        Serial.println("=======END TIMING INFO=======");
    }
}
