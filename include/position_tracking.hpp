#include "drive_train.hpp"
#include "eigen.h"
#include "imu.hpp"
#include "lib/odometry.hpp"
#include "scheduler_task.hpp"

class PositionTrackingTask : public SchedulerTask {
  private:
    ImuTask *imu_task;
    DriveTrainTask *drive_train_task;

    float last_left_wheel_position;
    float last_right_wheel_position;

    float last_heading;

    OdometryModule odometry;

    Pose current_pose;

  public:
    PositionTrackingTask(ImuTask *imu_task, DriveTrainTask *drive_train_task);

    void setup();
    void loop();

    int get_frequency() const override { return 60; }

    Pose get_current_pose();
};