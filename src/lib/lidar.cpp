#include "lib/lidar.hpp"
#include "lib/crc.hpp"
#include <Eigen/Geometry>
#include <algorithm>

#define DEG_TO_RAD (std::numbers::pi / 180.0)
#define FULL_TURN (2.0 * std::numbers::pi)

float to_cartesian_ccw_angle(float sensor_angle) {
    float cartesian = (float)(std::numbers::pi / 2.0) - sensor_angle;
    return std::fmod(cartesian + FULL_TURN, FULL_TURN);
}

std::variant<std::optional<LidarResponseData>, PacketParseError>
parse_packet(etl::span<uint8_t> packet) {
    if (packet[0] != HEADER || packet[1] != VERLEN) {
        return std::nullopt;
    }

    if (packet.size() != PACKET_SIZE) {
        return PacketParseError::IncorrectSize;
    }

    LidarPacket data;

    std::memcpy(&data, packet.data(), PACKET_SIZE);

    uint8_t packet_crc = calculate_crc(packet.first(PACKET_SIZE - 1));

    if (packet_crc != data.crc8) {
        return PacketParseError::CrcError;
    }

    float start_angle = DEG_TO_RAD * (float)(data.start_angle % 36000) / 100.0;
    float end_angle = DEG_TO_RAD * (float)(data.end_angle % 36000) / 100.0;

    float angle_diff = std::fmod(end_angle - start_angle + FULL_TURN, FULL_TURN);

    float increment = angle_diff / (float)(POINTS_PER_PACK - 1);

    std::array<LidarResponsePoint, POINTS_PER_PACK> response_points;

    for (int i = 0; i < POINTS_PER_PACK; i++) {
        float angle = to_cartesian_ccw_angle(start_angle + (float)i * increment);

        Eigen::Vector2f right((float)data.points[i].distance / 1e3, 0.0);

        Eigen::Rotation2D<float> rot(angle);

        response_points[i].position = rot * right;
        response_points[i].angle = angle;
        response_points[i].intensity = data.points[i].itensity;
    }

    LidarResponseData response = {
        .angular_velocity = DEG_TO_RAD * (float)data.speed,
        .start_angle = start_angle,
        .end_angle = end_angle,
        .points = response_points,
    };

    return response;
}

std::variant<std::optional<LidarResponseData>, PacketParseError>
LidarDataReader::read_span(etl::span<uint8_t> span) {
    if (this->data_buffer.size() + span.size() > MAX_BUFFER_SIZE) {
        return PacketParseError::BufferFull;
    }

    this->data_buffer.insert(this->data_buffer.end(), span.begin(), span.end());

    while (this->data_buffer.size() >= PACKET_SIZE) {
        if (this->data_buffer[0] != HEADER || this->data_buffer[1] != VERLEN) {
            this->data_buffer.erase(this->data_buffer.begin());
        } else {
            std::array<uint8_t, PACKET_SIZE> packet_data;
            std::copy_n(this->data_buffer.begin(), PACKET_SIZE, packet_data.begin());
            this->data_buffer.erase(this->data_buffer.begin(),
                                    this->data_buffer.begin() + PACKET_SIZE);

            return parse_packet({packet_data.data(), packet_data.size()});
        }
    }

    return std::nullopt;
}