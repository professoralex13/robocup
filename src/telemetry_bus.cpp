#include "telemetry_bus.hpp"

#include <cstring>

namespace telemetry {

namespace {

constexpr uint8_t FRAME_PREAMBLE[4] = {0xAB, 0xCD, 0xEF, 0x42};
constexpr uint8_t FRAME_VERSION = 1;

enum class FrameType : uint8_t {
    Values = 1,
    LidarPoints = 2,
};

struct __attribute__((packed)) FrameHeader {
    uint8_t preamble[4];
    uint8_t frame_type;
    uint8_t version;
    uint16_t payload_len;
};

struct __attribute__((packed)) ValueEntry {
    uint16_t key;
    uint8_t type;
    uint8_t flags;
    uint32_t payload;
};

struct __attribute__((packed)) LidarPointEntry {
    int16_t x_mm;
    int16_t y_mm;
    uint8_t intensity;
    uint8_t flags;
};

constexpr size_t MAX_PENDING_VALUES = 128;

Stream *TELEMETRY_PORT = &Serial;
ValueEntry PENDING_VALUES[MAX_PENDING_VALUES];
size_t PENDING_COUNT = 0;

template <typename T> uint32_t to_u32_bits(T value) {
    uint32_t bits = 0;
    static_assert(sizeof(T) <= sizeof(bits));
    memcpy(&bits, &value, sizeof(T));
    return bits;
}

void write_frame(FrameType frame_type, const uint8_t *payload, uint16_t payload_len) {
    FrameHeader header = {
        .preamble = {FRAME_PREAMBLE[0], FRAME_PREAMBLE[1], FRAME_PREAMBLE[2], FRAME_PREAMBLE[3]},
        .frame_type = static_cast<uint8_t>(frame_type),
        .version = FRAME_VERSION,
        .payload_len = payload_len,
    };

    TELEMETRY_PORT->write(reinterpret_cast<uint8_t *>(&header), sizeof(header));
    TELEMETRY_PORT->write(payload, payload_len);
}

bool upsert_value(uint16_t key, ValueType type, uint32_t payload) {
    for (size_t i = 0; i < PENDING_COUNT; i++) {
        if (PENDING_VALUES[i].key == key) {
            PENDING_VALUES[i].type = static_cast<uint8_t>(type);
            PENDING_VALUES[i].payload = payload;
            return true;
        }
    }

    if (PENDING_COUNT >= MAX_PENDING_VALUES) {
        return false;
    }

    PENDING_VALUES[PENDING_COUNT++] = {
        .key = key,
        .type = static_cast<uint8_t>(type),
        .flags = 0,
        .payload = payload,
    };

    return true;
}

} // namespace

void begin(Stream *serial_port) {
    if (serial_port != nullptr) {
        TELEMETRY_PORT = serial_port;
    }
}

bool publish_f32(uint16_t key, float value) {
    return upsert_value(key, ValueType::Float32, to_u32_bits(value));
}

bool publish_i32(uint16_t key, int32_t value) {
    return upsert_value(key, ValueType::Int32, to_u32_bits(value));
}

bool publish_u32(uint16_t key, uint32_t value) {
    return upsert_value(key, ValueType::UInt32, to_u32_bits(value));
}

bool publish_bool(uint16_t key, bool value) {
    return upsert_value(key, ValueType::Bool, value ? 1 : 0);
}

size_t flush_values(uint16_t max_entries_per_frame) {
    if (PENDING_COUNT == 0 || max_entries_per_frame == 0) {
        return 0;
    }

    if (max_entries_per_frame > 32) {
        max_entries_per_frame = 32;
    }

    size_t to_send = PENDING_COUNT;
    if (to_send > max_entries_per_frame) {
        to_send = max_entries_per_frame;
    }

    const uint16_t payload_len =
        static_cast<uint16_t>(sizeof(uint16_t) + to_send * sizeof(ValueEntry));
    uint8_t payload[sizeof(uint16_t) + 32 * sizeof(ValueEntry)] = {0};

    uint16_t count = static_cast<uint16_t>(to_send);
    memcpy(payload, &count, sizeof(count));
    memcpy(payload + sizeof(uint16_t), PENDING_VALUES, to_send * sizeof(ValueEntry));

    write_frame(FrameType::Values, payload, payload_len);

    size_t remaining = PENDING_COUNT - to_send;
    if (remaining > 0) {
        memmove(PENDING_VALUES, PENDING_VALUES + to_send, remaining * sizeof(ValueEntry));
    }
    PENDING_COUNT = remaining;

    return to_send;
}

void publish_lidar_points(const std::array<LidarResponsePoint, POINTS_PER_PACK> &points) {
    LidarPointEntry packed_points[POINTS_PER_PACK] = {};

    for (size_t i = 0; i < POINTS_PER_PACK; i++) {
        packed_points[i] = {
            .x_mm = static_cast<int16_t>(points[i].position.x() * 1000.0f),
            .y_mm = static_cast<int16_t>(points[i].position.y() * 1000.0f),
            .intensity = points[i].intensity,
            .flags = 0,
        };
    }

    constexpr uint16_t payload_len = sizeof(uint16_t) + POINTS_PER_PACK * sizeof(LidarPointEntry);
    uint8_t payload[payload_len] = {0};

    uint16_t count = POINTS_PER_PACK;
    memcpy(payload, &count, sizeof(count));
    memcpy(payload + sizeof(uint16_t), packed_points, POINTS_PER_PACK * sizeof(LidarPointEntry));

    write_frame(FrameType::LidarPoints, payload, payload_len);
}

} // namespace telemetry
