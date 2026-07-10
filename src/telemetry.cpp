#include "telemetry.hpp"
#include "Arduino.h"
#include <wiring.h>

const uint8_t TELEMETRY_HEADER[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

#define NUM_TRANSMIT_POINTS 500

struct __attribute__((packed)) LidarTelemetryPoint {
    int16_t x;
    int16_t y;
    uint8_t intensity;
};

struct __attribute__((packed)) TelemetryPacket {
    LidarTelemetryPoint lidar_points[NUM_TRANSMIT_POINTS];
    float heading;
    float pitch;
    float left_wheel_velocity;
    float right_wheel_velocity;

    float position_x;
    float position_y;
    float position_uncertainty;
};

TelemetryTask::TelemetryTask(LidarTask *lidar_task, ImuTask *imu_task,
                             DriveTrainTask *drive_train_task,
                             PositionTrackingTask *position_tracking_task)
    : SchedulerTask("telemetry_task"), lidar_task(lidar_task), imu_task(imu_task),
      drive_train_task(drive_train_task), position_tracking_task(position_tracking_task) {}

void TelemetryTask::setup() { Serial.begin(921600); }

void TelemetryTask::loop() {
    TelemetryPacket packet;

    for (int i = 0; i < NUM_TRANSMIT_POINTS; i++) {
        if (this->lidar_task->points.size() <= i) {
            break;
        }

        auto point = this->lidar_task->points[this->lidar_task->points.size() - i - 1];
        packet.lidar_points[i].x = (int16_t)(point.position.x() * 1000.0);
        packet.lidar_points[i].y = (int16_t)(point.position.y() * 1000.0);
        packet.lidar_points[i].intensity = point.intensity;
    }

    auto angles = imu_task->get_euler_angles();
    Pose localization_pose = position_tracking_task->get_current_pose();

    packet.heading = localization_pose.heading;
    packet.pitch = angles.x();

    packet.left_wheel_velocity = drive_train_task->get_left_wheel_velocity();
    packet.right_wheel_velocity = drive_train_task->get_right_wheel_velocity();

    packet.position_x = localization_pose.position.x();
    packet.position_y = localization_pose.position.y();
    packet.position_uncertainty = position_tracking_task->get_position_uncertainty();

    Serial.write(TELEMETRY_HEADER, sizeof(TELEMETRY_HEADER));
    Serial.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}
