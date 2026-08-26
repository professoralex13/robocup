#pragma once

#include "scheduler_task.hpp"
#include "tasks/lidar.hpp"

class LidarProcessingTask : public SchedulerTask {
  private:
    LidarTask *lidar_reader_task;

  public:
    LidarProcessingTask(LidarTask *lidar_reader_task);

    void setup();
    void loop();

    int get_frequency() const override { return LIDAR_PROCESSING_FREQ; }
};