#pragma once

#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"
#include "tasks/position_tracking.hpp"

class LidarProcessingTask : public SchedulerTask {
  private:
    LidarTask *lidar_reader_task;
    PositionTrackingTask *position_tracking_task;

  public:
    LidarProcessingTask(LidarTask *lidar_reader_task, PositionTrackingTask *position_tracking_task);

    void setup();
    void loop();

    int get_frequency() const override { return LIDAR_PROCESSING_FREQ; }
};