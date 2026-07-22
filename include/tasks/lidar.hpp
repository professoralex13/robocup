#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <config.hpp>
#include <etl/vector.h>

class LidarTask : public SchedulerTask {
  private:
    LidarDataReader reader;

  public:
    LidarTask();

    void setup();
    void loop();

    int get_frequency() const override { return LIDAR_TASK_FREQ; }

    etl::vector<LidarResponsePoint, LIDAR_POINT_HISTORY_CAPACITY> points;
};