#pragma once

#include "drive_train.hpp"
#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <etl/deque.h>

#define LIDAR_POINTS_HISTORY 1500

class UserCommandTask : public SchedulerTask {
  private:
    DriveTrainTask *drive_train_task;

  public:
    UserCommandTask(DriveTrainTask *drive_train_task);

    void setup();
    void loop();

    int get_frequency() const override { return 50; }
};