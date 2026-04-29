#include "lidar.hpp"
#include "Arduino.h"

LidarTask::LidarTask() : SchedulerTask("lidar_task") {}

void LidarTask::setup() { Serial2.begin(230400); }

void LidarTask::loop() {
    uint8_t buffer[4]; // Teensy 4.0 UART FIFO buffer is only 4 bytes maximum

    int length = Serial2.available();

    length = Serial2.readBytes(buffer, length);

    auto response = this->reader.read_span({buffer, length});

    std::visit(
        [this](auto &&arg) {
            using T = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<T, std::optional<LidarResponseData>>) {
                if (arg.has_value()) {
                    for (int i = 0; i < POINTS_PER_PACK; i++) {
                        if (this->points.full()) {
                            this->points.pop_front();
                        }

                        this->points.push_back((*arg).points[i]);
                    }
                }
            } else {
                log_err("Failed to parse LIDAR data");
            }
        },
        response);
}
