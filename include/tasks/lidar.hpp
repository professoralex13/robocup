#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <config.hpp>
#include <etl/deque.h>

class LidarTask : public SchedulerTask {
  private:
    LidarDataReader reader;

  public:
    LidarTask();

    void setup();
    void loop();

    int get_frequency() const override { return LIDAR_TASK_FREQ; }

    etl::deque<LidarResponsePoint, MAX_ALLOWABLE_LIDAR_POINTS> points;
};