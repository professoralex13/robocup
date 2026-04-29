#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <etl/deque.h>

#define MAX_LIDAR_POINTS 500

class LidarTask : public SchedulerTask {
  private:
    LidarDataReader reader;

  public:
    LidarTask();

    void setup();
    void loop();

    int get_frequency() const override { return 500; }

    etl::deque<LidarResponsePoint, MAX_LIDAR_POINTS> points;
};