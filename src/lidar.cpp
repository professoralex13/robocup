#include "lidar.hpp"
#include "Arduino.h"
#include <algorithm>

LidarTask::LidarTask() : SchedulerTask("lidar_task") {}

static uint8_t SERIAL_MEMORY[200];

void LidarTask::setup() {
    Serial2.begin(230400);
    Serial2.addMemoryForRead(SERIAL_MEMORY, sizeof(SERIAL_MEMORY));
    Serial2.setTimeout(0);
}

void LidarTask::loop() {
    static uint8_t buffer[100];

    int available = Serial2.available();
    if (available <= 0) {
        return;
    }

    size_t to_read = std::min((size_t)available, sizeof(buffer));

    int length = Serial2.readBytes(buffer, to_read);
    if (length <= 0) {
        return;
    }

    auto response = this->reader.read_span({buffer, length});

    std::visit(
        [this](auto &&arg) {
            using T = std::decay_t<decltype(arg)>;

            if constexpr (std::is_same_v<T, std::optional<LidarResponseData>>) {
                if (arg.has_value()) {
                    for (int i = 0; i < POINTS_PER_PACK; i++) {
                        if ((*arg).points[i].position.norm() > 12.0) {
                            continue;
                        }

                        if (this->points.full()) {
                            this->points.pop_front();
                        }

                        this->points.push_back((*arg).points[i]);
                    }
                }
            } else if constexpr (std::is_same_v<T, PacketParseError>) {
                switch (arg) {
                case PacketParseError::BufferFull:
                    log_err("Serial Buffer Full");
                    break;
                case PacketParseError::IncorrectSize:
                    log_err("Incorrect data size");
                    break;
                case PacketParseError::CrcError:
                    // These are common so we ignore
                    // log_err("CRC Error");
                    break;
                }
            }
        },
        response);
}
