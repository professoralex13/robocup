#include "telemetry.hpp"
#include "Arduino.h"
#include <wiring.h>

TelemetryTask::TelemetryTask(LidarTask *lidar_task)
    : SchedulerTask("telemetry_task"), lidar_task(lidar_task) {}

void TelemetryTask::setup() { Serial7.begin(115200); }

const uint8_t TELEMETRY_HEADER[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

struct __attribute__((packed)) LidarTelemetryPoint {
    int16_t x;
    int16_t y;
    uint8_t intensity;
};

struct __attribute__((packed)) TelemetryPacket {
    LidarTelemetryPoint lidar_points[MAX_LIDAR_POINTS];
};

void TelemetryTask::loop() {
    if (!this->lidar_task->points.full()) {
        return;
    }

    log("Sending Packet");

    TelemetryPacket packet;

    for (int i = 0; i < MAX_LIDAR_POINTS; i++) {
        auto point = this->lidar_task->points[i];
        packet.lidar_points[i].x = (int16_t)(point.position.x() * 1000.0);
        packet.lidar_points[i].y = (int16_t)(point.position.y() * 1000.0);
        packet.lidar_points[i].intensity = 250;
        // packet.lidar_points[i].intensity = (uint16_t)i * 256 / MAX_LIDAR_POINTS;
    }

    Serial7.write(TELEMETRY_HEADER, sizeof(TELEMETRY_HEADER));
    Serial7.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}
