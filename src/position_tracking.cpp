#include "position_tracking.hpp"
#include "lib/odometry.hpp"
#include "utils.hpp"
#undef B1
#include <Eigen/Geometry>

namespace {

constexpr float DRIVE_WIDTH_MM = 255.0f;
constexpr float FIELD_WIDTH_X_METERS = 2.4f;
constexpr float FIELD_HEIGHT_Y_METERS = 4.9f;

constexpr float WHEEL_RADIUS_MM = 31.0f * (54.0f / 18.0f);
constexpr float MM_TO_METERS = 1e-3f;

constexpr float INITIAL_X = 1.2f;
constexpr float INITIAL_Y = 0.4f;

} // namespace

PositionTrackingTask::PositionTrackingTask(ImuTask *imu_task, DriveTrainTask *drive_train_task,
                                           LidarTask *lidar_task)
    : SchedulerTask("position_tracking"), imu_task(imu_task), drive_train_task(drive_train_task),
      lidar_task(lidar_task),
      odometry(OdometryModule({{
          {Eigen::Vector2f(-DRIVE_WIDTH_MM / 2.0f, 0.0f), Eigen::Vector2f(0.0f, 1.0f)},
          {Eigen::Vector2f(DRIVE_WIDTH_MM / 2.0f, 0.0f), Eigen::Vector2f(0.0f, 1.0f)},
      }})),
      mcl(FieldMap::make_rectangle(FIELD_WIDTH_X_METERS, FIELD_HEIGHT_Y_METERS)) {}

void PositionTrackingTask::setup() {
    this->last_left_wheel_position = drive_train_task->get_left_wheel_position();
    this->last_right_wheel_position = drive_train_task->get_right_wheel_position();
    this->last_heading = imu_task->get_euler_angles().y();

    this->current_pose = {.position = Eigen::Vector2f(INITIAL_X, INITIAL_Y),
                          .heading = this->last_heading};

    this->mcl.set_initial_pose(this->current_pose, 0.06f, 0.05f);
    this->initialized = true;
}

void PositionTrackingTask::loop() {
    if (!this->initialized) {
        this->setup();
    }

    float left_wheel_position = drive_train_task->get_left_wheel_position();
    float right_wheel_position = drive_train_task->get_right_wheel_position();

    float left_change = left_wheel_position - this->last_left_wheel_position;
    float right_change = right_wheel_position - this->last_right_wheel_position;

    float wheel_travels[2] = {left_change * WHEEL_RADIUS_MM, right_change * WHEEL_RADIUS_MM};

    float current_heading = imu_task->get_euler_angles().y();

    float heading_change = diff_angle(last_heading, current_heading);

    Eigen::Vector2f robot_travel = this->odometry.compute_travel(wheel_travels, heading_change);

    robot_travel *= MM_TO_METERS;

    this->mcl.predict(robot_travel, heading_change);
    this->mcl.update_beam_model(this->lidar_task->points);

    this->current_pose = this->mcl.get_estimated_pose();

    this->last_left_wheel_position = left_wheel_position;
    this->last_right_wheel_position = right_wheel_position;
    this->last_heading = current_heading;
}

Pose PositionTrackingTask::get_current_pose() { return this->current_pose; }

float PositionTrackingTask::get_position_uncertainty() {
    return this->mcl.get_position_uncertainty();
}