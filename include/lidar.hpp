#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <etl/deque.h>

#define LIDAR_POINTS_HISTORY 1500

class LidarTask : public SchedulerTask {
  private:
    LidarDataReader reader;

  public:
    LidarTask();

    void setup();
    void loop();

    int get_frequency() const override { return 500; }

    etl::deque<LidarResponsePoint, LIDAR_POINTS_HISTORY> points;
};