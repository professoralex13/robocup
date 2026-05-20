#include "drive_train.hpp"
#include <wiring.h>

DriveTrainTask::DriveTrainTask() : SchedulerTask("drive_train_task") {}

void DriveTrainTask::setup() {
    left_motor.attach(LEFT_MOTOR_CONTROL_PIN);
    right_motor.attach(RIGHT_MOTOR_CONTROL_PIN);
}

#define MAX_REVERSE 1050
#define MAX_FORWARD 1950

void DriveTrainTask::loop() {
    left_motor.writeMicroseconds(map(this->left_command, -1.0, 1.0, MAX_FORWARD, MAX_REVERSE));
    right_motor.writeMicroseconds(map(this->right_command, -1.0, 1.0, MAX_REVERSE, MAX_FORWARD));

    uint32_t timestamp = micros();

    float dt = (float)(timestamp - last_timestamp) / 1e6;

    last_timestamp = timestamp;

    int left_ticks = -left_encoder.read();
    int right_ticks = right_encoder.read();

    this->left_velocity = RADIANS_PER_TICK * (float)(left_ticks - last_left_ticks) / dt;
    this->right_velocity = RADIANS_PER_TICK * (float)(right_ticks - last_right_ticks) / dt;

    last_left_ticks = left_ticks;
    last_right_ticks = right_ticks;
}

float DriveTrainTask::get_left_wheel_velocity() { return this->left_velocity; }

float DriveTrainTask::get_right_wheel_velocity() { return this->right_velocity; }

void DriveTrainTask::set_commands(float left, float right) {
    this->left_command = left;
    this->right_command = right;
}
