#pragma once

#include "etl/span.h"
#include "etl/vector.h"
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>

#define POINTS_PER_PACK 12

#define HEADER 0x54
#define VERLEN 0x2C

struct __attribute__((packed)) LidarPacketPoint {
    uint16_t distance;
    uint8_t itensity;
};

struct __attribute__((packed)) LidarPacket {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPacketPoint points[POINTS_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
};

#define PACKET_SIZE sizeof(LidarPacket)

constexpr size_t LIDAR_POINT_HISTORY_CAPACITY = 500;

struct LidarResponsePoint {
    Eigen::Vector2f position;
    float angle;
    uint8_t intensity;
};

struct LidarResponseData {
    float angular_velocity;
    float start_angle;
    float end_angle;
    std::array<LidarResponsePoint, POINTS_PER_PACK> points;
};

enum class PacketParseError {
    IncorrectSize,
    BufferFull,
    CrcError,
};

#define MAX_BUFFER_SIZE 400

class LidarDataReader {
  private:
    etl::vector<uint8_t, MAX_BUFFER_SIZE> data_buffer;

  public:
    std::variant<std::optional<LidarResponseData>, PacketParseError>
    read_span(etl::span<uint8_t> span);
};