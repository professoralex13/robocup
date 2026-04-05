#include "lidar.hpp"

LidarTask::LidarTask(HardwareSerialIMXRT serial) : SchedulerTask("lidar_task"), serial(serial) {}

void LidarTask::setup() {}

void LidarTask::loop() {
    char buffer[50];

    size_t length = serial.readBytes(buffer, 50);
}
