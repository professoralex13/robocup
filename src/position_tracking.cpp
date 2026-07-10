#include "position_tracking.hpp"
#include "lib/odometry.hpp"
#include "utils.hpp"
#undef B1
#include <Eigen/Geometry>

#define DRIVE_WIDTH 255

PositionTrackingTask::PositionTrackingTask(ImuTask *imu_task, DriveTrainTask *drive_train_task)
    : SchedulerTask("position_tracking"), imu_task(imu_task), drive_train_task(drive_train_task),
      odometry(OdometryModule({{
          {Eigen::Vector2f(-DRIVE_WIDTH / 2.0, 0.0), Eigen::Vector2f(0.0, 1.0)},
          {Eigen::Vector2f(DRIVE_WIDTH / 2.0, 0.0), Eigen::Vector2f(0.0, 1.0)},
      }})) {}

void PositionTrackingTask::setup() {}

#define WHEEL_RADIUS (31.0 * (54.0 / 18.0))

void PositionTrackingTask::loop() {
    float left_wheel_position = drive_train_task->get_left_wheel_position();
    float right_wheel_position = drive_train_task->get_right_wheel_position();

    float left_change = left_wheel_position - this->last_left_wheel_position;
    float right_change = right_wheel_position - this->last_right_wheel_position;

    float wheel_travels[2] = {left_change * WHEEL_RADIUS, right_change * WHEEL_RADIUS};

    float current_heading = -imu_task->get_euler_angles().y();

    float heading_change = diff_angle(last_heading, current_heading);

    Eigen::Vector2f robot_travel = this->odometry.compute_travel(wheel_travels, heading_change);

    this->current_pose.position += Eigen::Rotation2Df(this->current_pose.heading) * robot_travel;
    this->current_pose.heading = wrap_heading(this->current_pose.heading + heading_change);

    this->last_left_wheel_position = left_wheel_position;
    this->last_right_wheel_position = right_wheel_position;
    this->last_heading = current_heading;
}

Pose PositionTrackingTask::get_current_pose() { return this->current_pose; }