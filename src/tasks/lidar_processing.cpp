#include "tasks/lidar_processing.hpp"
#include "lib/lidar_processing.hpp"
#include "telemetry_bus.hpp"

LidarProcessingTask::LidarProcessingTask(LidarTask *lidar_reader_task)
    : SchedulerTask("lidar_processing"), lidar_reader_task(lidar_reader_task) {}

void LidarProcessingTask::setup() {}

void LidarProcessingTask::loop() {
    auto points = lidar_reader_task->get_points();

    LidarProcessing processing;

    auto result = processing.process_points(points);

    telemetry::publish_lidar_points(points);
    telemetry::publish_lidar_processing(result);

    // if (now >= next_lidar_telemetry_publish) {
    //     LidarProcessing processing;

    //     next_lidar_telemetry_publish = now + LIDAR_TELEMETRY_PERIOD_MICROS;
    // }
}
