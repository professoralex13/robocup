#include "drive_train.hpp"
#include "imu.hpp"
#include "lib/lidar.hpp"
#include "lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <etl/deque.h>

class TelemetryTask : public SchedulerTask {
  private:
    LidarTask *lidar_task;
    ImuTask *imu_task;
    DriveTrainTask *drive_train_task;

  public:
    TelemetryTask(LidarTask *lidar_task, ImuTask *imu_task, DriveTrainTask *drive_train_task);

    void setup();
    void loop();

    int get_frequency() const override { return 3; }
};