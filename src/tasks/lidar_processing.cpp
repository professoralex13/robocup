#include "tasks/lidar_processing.hpp"
#include "lib/lidar_processing.hpp"
#include "telemetry_bus.hpp"

LidarProcessingTask::LidarProcessingTask(LidarTask *lidar_reader_task,
                                         PositionTrackingTask *position_tracking_task)
    : SchedulerTask("lidar_processing"), lidar_reader_task(lidar_reader_task),
      position_tracking_task(position_tracking_task) {}

void LidarProcessingTask::setup() {}

void LidarProcessingTask::loop() {
    auto points = lidar_reader_task->get_points();
    auto pose = position_tracking_task->get_current_pose();

    LidarProcessing processing;

    auto result = processing.process_points(points, pose);

    telemetry::publish_lidar_points(result.transformed_points);
    telemetry::publish_lidar_processing(result);

    // if (now >= next_lidar_telemetry_publish) {
    //     LidarProcessing processing;

    //     next_lidar_telemetry_publish = now + LIDAR_TELEMETRY_PERIOD_MICROS;
    // }
}
