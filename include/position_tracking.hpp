#include "drive_train.hpp"
#include "eigen.h"
#include "imu.hpp"
#include "lib/monte_carlo_localization.hpp"
#include "lib/odometry.hpp"
#include "lidar.hpp"
#include "scheduler_task.hpp"

class PositionTrackingTask : public SchedulerTask {
  private:
    ImuTask *imu_task;
    DriveTrainTask *drive_train_task;
    LidarTask *lidar_task;

    float last_left_wheel_position;
    float last_right_wheel_position;

    float last_heading;

    bool initialized = false;

    OdometryModule odometry;
    MonteCarloLocalization mcl;

    Pose current_pose;

  public:
    PositionTrackingTask(ImuTask *imu_task, DriveTrainTask *drive_train_task,
                         LidarTask *lidar_task);

    void setup();
    void loop();

    int get_frequency() const override { return 60; }

    Pose get_current_pose();
    float get_position_uncertainty();
};