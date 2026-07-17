#include "motion_control_task.hpp"

#define DRIVE_KP 15e-1
#define DRIVE_KI 0 // 20e-4

#define TURN_KP 4.0e-1
#define TURN_KI 0
#define TURN_KD 0 // 1.2e-1

MotionControlTask::MotionControlTask(DriveTrainTask *drive_train_task,
                                     PositionTrackingTask *position_tracking_task)
    : SchedulerTask("motion_control"), drive_train_task(drive_train_task),
      position_tracking_task(position_tracking_task), pure_pursuit(0.1),
      pid_drive(PIDController(DRIVE_KP, DRIVE_KI, 0, 10)
                    .with_output_limits(-1, 1)
                    .with_integral_bounds(-100, 100)),
      pid_turn(PIDController(TURN_KP, TURN_KI, TURN_KD, 3 * DEG_TO_RAD)
                   .with_output_limits(-1, 1)
                   .with_integral_bounds(-30 * DEG_TO_RAD, 30 * DEG_TO_RAD)) {}

void MotionControlTask::setup() {
    pure_pursuit.set_current_path({{1.2, 0.4}, {1.2, 2.0}, {2.0, 2.0}});
}

void MotionControlTask::loop() {
    auto pose = position_tracking_task->get_current_pose();

    auto [drive_error, turn_error] = pure_pursuit.compute_errors(pose);

    this->drive_error = drive_error;
    this->turn_error = turn_error;

    float drive_output = pid_drive.update(drive_error);
    float turn_output = pid_turn.update(turn_error);

    float left_drive = drive_output + turn_output;
    float right_drive = drive_output - turn_output;
    float largest_cmd = fabs(fmax(left_drive, right_drive));

    if (largest_cmd > 1.0) {
        left_drive = left_drive / largest_cmd;
        right_drive = right_drive / largest_cmd;
    }

    drive_train_task->set_commands(left_drive, right_drive);
}