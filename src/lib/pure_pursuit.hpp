#include "lib/pure_pursuit.hpp"
#include <utils.hpp>

using Eigen::Vector2f;

const float TURN_DEADZONE = 0.1; // TODO: Tune this

PurePursuit::PurePursuit(float look_ahead_distance) : look_ahead_distance(look_ahead_distance) {}

void PurePursuit::set_current_path(std::vector<Vector2f> positions, bool backwards) {
    current_path = positions;
    drive_path_backwards = backwards;
}

void PurePursuit::set_drive_direction(bool backwards) { drive_path_backwards = backwards; }

std::tuple<float, float> PurePursuit::compute_errors(Pose current_pose) {
    auto current_position = current_pose.position;

    float turn_error = 0;
    float drive_error = 0;

    if (current_path.size() >= 1) {
        auto current_direction = current_pose.get_direction_vector();

        float current_heading = current_pose.heading;

        if (drive_path_backwards) {
            current_heading = diff_angle(std::numbers::pi, current_heading);
        }

        // Apply regular pure pursuit
        Vector2f last_point;
        Vector2f next_point;

        if (current_path.size() == 1) {
            last_point = current_position;
            next_point = current_path[0];
        } else {
            last_point = current_path[0];
            next_point = current_path[1];
        }

        auto lookahead_point = get_snapped_radius_target(current_position, look_ahead_distance,
                                                         last_point, next_point);

        auto closest_point = get_closest_point(current_position, last_point, next_point);

        // Initialize Drive Error as distance to lookahead point plus distance from lookahead point
        // to next point
        drive_error =
            (current_position - lookahead_point).norm() + (lookahead_point, next_point).norm();

        // Initialize remaining_distance as distance from the closest point on the current segment,
        // to the end of the current segment
        remaining_distance = (closest_point - next_point).norm();

        // Loop through the remaining path segments adding that distance to both remaining distance
        // and drive error
        for (int i = 1; i < current_path.size() - 1; i++) {
            float segment_length = (current_path[i] - current_path[i + 1]).norm();

            remaining_distance += segment_length;
            drive_error += segment_length;
        }

        drive_error =
            drive_error * (lookahead_point - current_position).normalized().dot(current_direction);

        auto direction = lookahead_point - current_position;

        turn_error = diff_angle(current_heading, std::atan2(direction.x(), direction.y()));

        float distance_to_next_point = (next_point - current_position).norm();

        // Only enable turn PID outside a certain distance of target point (this should only
        // matter when approaching the last point in the current path)
        if (distance_to_next_point <= TURN_DEADZONE) {
            turn_error = 0;
        }

        // next_point_distance may jump around, but will not go far
        // below look_ahead_distance until approaching the final point

        if (current_path.size() >= 2 && distance_to_next_point <= look_ahead_distance) {
            current_path.erase(current_path.begin());
        }
    }

    return std::make_tuple(drive_error, turn_error);
}
