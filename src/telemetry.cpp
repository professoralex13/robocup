#include "telemetry.hpp"
#include "Arduino.h"
#include <wiring.h>

TelemetryTask::TelemetryTask(LidarTask *lidar_task, ImuTask *imu_task,
                             DriveTrainTask *drive_train_task)
    : SchedulerTask("telemetry_task"), lidar_task(lidar_task), imu_task(imu_task),
      drive_train_task(drive_train_task) {}

void TelemetryTask::setup() { Serial7.begin(115200); }

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
};

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

    packet.heading = angles.y();
    packet.pitch = angles.x();

    packet.left_wheel_velocity = drive_train_task->get_left_wheel_velocity();
    packet.right_wheel_velocity = drive_train_task->get_right_wheel_velocity();

    Serial7.write(TELEMETRY_HEADER, sizeof(TELEMETRY_HEADER));
    Serial7.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}
