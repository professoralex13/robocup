#include "lidar.hpp"
#include "Arduino.h"

LidarTask::LidarTask(HardwareSerialIMXRT serial) : SchedulerTask("lidar_task"), serial(serial) {}

void LidarTask::setup() { serial.begin(230400); }

void LidarTask::loop() {
    uint8_t buffer[4]; // Teensy 4.0 UART FIFO buffer is only 4 bytes maximum

    int length = serial.available();

    length = serial.readBytes(buffer, length);

    auto response = this->reader.read_span({buffer, length});

    std::visit(
        [this](auto &&arg) {
            using T = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<T, std::optional<LidarResponseData>>) {
                if (arg.has_value()) {
                    Serial.printf("Start: %d, end: %d\n", (int)((*arg).start_angle / DEG_TO_RAD),
                                  (int)((*arg).end_angle / DEG_TO_RAD));
                }
            } else {
                log_err("Failed to parse LIDAR data");
            }
        },
        response);
}
