#include "lidar.hpp"
#include "Arduino.h"
#undef B1
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

LidarTask::LidarTask() : SchedulerTask("lidar_task") {}

static uint8_t SERIAL_MEMORY[200];

namespace {

// Lidar extrinsic in robot frame.
// +X is robot-right, +Y is robot-forward, yaw is compass-style clockwise-positive.
constexpr float LIDAR_OFFSET_X_METERS = 0.0f;
constexpr float LIDAR_OFFSET_Y_METERS = 0.0f;
constexpr float LIDAR_YAW_OFFSET_RADIANS = 0.0f;

// Ignored angular range in robot frame around +Y forward axis.
// 0 rad = forward, +pi/2 = right, -pi/2 = left.
constexpr bool LIDAR_IGNORE_ANGLE_ENABLED = true;
constexpr float LIDAR_DEG_TO_RAD = 0.01745329252f;
constexpr float LIDAR_IGNORE_BOUND_A_RADIANS = -135.0f * LIDAR_DEG_TO_RAD;
constexpr float LIDAR_IGNORE_BOUND_B_RADIANS = 90.0f * LIDAR_DEG_TO_RAD;

constexpr float WRAP_TWO_PI = 6.28318530718f;

float wrap_angle_positive(float angle) {
    float wrapped = fmodf(angle, WRAP_TWO_PI);
    if (wrapped < 0.0f) {
        wrapped += WRAP_TWO_PI;
    }
    return wrapped;
}

} // namespace

void LidarTask::set_mount_pose_in_robot_frame(const Eigen::Vector2f &position, float yaw) {
    this->mount_position_in_robot_frame = position;
    this->mount_yaw_in_robot_frame = yaw;
}

void LidarTask::set_ignored_angle_range(float start_radians, float end_radians) {
    this->ignore_angle_start_radians = start_radians;
    this->ignore_angle_end_radians = end_radians;
    this->ignore_angle_enabled = true;
}

void LidarTask::disable_ignored_angle_range() { this->ignore_angle_enabled = false; }

LidarResponsePoint LidarTask::to_robot_frame(const LidarResponsePoint &point) const {
    float heading = this->mount_yaw_in_robot_frame;
    float heading_sin = sinf(heading);
    float heading_cos = cosf(heading);

    Eigen::Vector2f rotated(heading_cos * point.position.x() + heading_sin * point.position.y(),
                            -heading_sin * point.position.x() + heading_cos * point.position.y());

    return {
        .position = rotated + this->mount_position_in_robot_frame,
        .intensity = point.intensity,
    };
}

bool LidarTask::is_angle_ignored(float angle_radians) const {
    if (!this->ignore_angle_enabled) {
        return false;
    }

    float angle = wrap_angle_positive(angle_radians);
    float start = wrap_angle_positive(this->ignore_angle_start_radians);
    float end = wrap_angle_positive(this->ignore_angle_end_radians);

    if (start <= end) {
        return angle >= start && angle <= end;
    }

    return angle >= start || angle <= end;
}

void LidarTask::setup() {
    Serial2.begin(230400);
    Serial2.addMemoryForRead(SERIAL_MEMORY, sizeof(SERIAL_MEMORY));
    Serial2.setTimeout(0);

    this->set_mount_pose_in_robot_frame({LIDAR_OFFSET_X_METERS, LIDAR_OFFSET_Y_METERS},
                                        LIDAR_YAW_OFFSET_RADIANS);

    if (LIDAR_IGNORE_ANGLE_ENABLED) {
        // Ignore the rear-facing arc bounded by -135° and +90°.
        // Using start=+90°, end=-135° selects the wraparound interval through 180°.
        this->set_ignored_angle_range(LIDAR_IGNORE_BOUND_B_RADIANS, LIDAR_IGNORE_BOUND_A_RADIANS);
    } else {
        this->disable_ignored_angle_range();
    }
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

                        LidarResponsePoint corrected = this->to_robot_frame((*arg).points[i]);
                        float point_angle = atan2f(corrected.position.x(), corrected.position.y());

                        if (this->is_angle_ignored(point_angle)) {
                            continue;
                        }

                        if (this->points.full()) {
                            this->points.pop_front();
                        }

                        this->points.push_back(corrected);
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
