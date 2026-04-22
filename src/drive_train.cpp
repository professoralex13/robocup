#include "drive_train.hpp"
#include <wiring.h>

DriveTrainTask::DriveTrainTask(int left_port, int right_port)
    : SchedulerTask("lidar_task"), left_port(left_port), right_port(right_port) {}

void DriveTrainTask::setup() {
    left_motor.attach(left_port);
    right_motor.attach(right_port);
}

#define MAX_REVERSE 1050
#define MAX_FORWARD 1950

void DriveTrainTask::loop() {
    left_motor.writeMicroseconds(map(this->left_command, -1.0, 1.0, MAX_REVERSE, MAX_FORWARD));
    right_motor.writeMicroseconds(map(this->left_command, -1.0, 1.0, MAX_REVERSE, MAX_FORWARD));
}

void DriveTrainTask::set_commands(float left, float right) {
    this->left_command = left;
    this->right_command = right;
}
