#pragma once

#include "config.hpp"
#include "lib/lidar.hpp"
#include <Arduino.h>
#include <array>
#include <cstdint>

namespace telemetry {

enum class ValueType : uint8_t {
    Float32 = 1,
    Int32 = 2,
    UInt32 = 3,
    Bool = 4,
};

enum Key : uint16_t {
    KEY_HEADING = 1,
    KEY_PITCH = 2,
    KEY_LEFT_WHEEL_VELOCITY = 3,
    KEY_RIGHT_WHEEL_VELOCITY = 4,
    KEY_POSITION_X = 5,
    KEY_POSITION_Y = 6,
    KEY_POSITION_UNCERTAINTY = 7,
};

void begin(Stream *serial_port = &Serial);

bool publish_f32(uint16_t key, float value);
bool publish_i32(uint16_t key, int32_t value);
bool publish_u32(uint16_t key, uint32_t value);
bool publish_bool(uint16_t key, bool value);

size_t flush_values(uint16_t max_entries_per_frame = 32);
void publish_lidar_points(const std::array<LidarResponsePoint, MAX_ALLOWABLE_LIDAR_POINTS> &points);

} // namespace telemetry
