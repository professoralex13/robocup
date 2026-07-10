#include "lib/field_map.hpp"
#include <algorithm>
#include <cmath>

namespace {

float cross_2d(const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
    return a.x() * b.y() - a.y() * b.x();
}

} // namespace

FieldMap FieldMap::make_square(float side_length_meters) {
    FieldMap map;

    float half_side = side_length_meters * 0.5f;

    Eigen::Vector2f bottom_left(-half_side, -half_side);
    Eigen::Vector2f bottom_right(half_side, -half_side);
    Eigen::Vector2f top_right(half_side, half_side);
    Eigen::Vector2f top_left(-half_side, half_side);

    map.add_wall(bottom_left, bottom_right);
    map.add_wall(bottom_right, top_right);
    map.add_wall(top_right, top_left);
    map.add_wall(top_left, bottom_left);

    return map;
}

FieldMap FieldMap::make_rectangle(float width_x_meters, float height_y_meters) {
    FieldMap map;

    Eigen::Vector2f bottom_left(0.0f, 0.0f);
    Eigen::Vector2f bottom_right(width_x_meters, 0.0f);
    Eigen::Vector2f top_right(width_x_meters, height_y_meters);
    Eigen::Vector2f top_left(0.0f, height_y_meters);

    map.add_wall(bottom_left, bottom_right);
    map.add_wall(bottom_right, top_right);
    map.add_wall(top_right, top_left);
    map.add_wall(top_left, bottom_left);

    return map;
}

bool FieldMap::add_wall(const Eigen::Vector2f &start, const Eigen::Vector2f &end) {
    if (this->num_wall_segments >= this->wall_segments.size()) {
        return false;
    }

    this->wall_segments[this->num_wall_segments++] = {
        .start = start,
        .end = end,
    };

    return true;
}

void FieldMap::clear() { this->num_wall_segments = 0; }

float FieldMap::raycast(const Eigen::Vector2f &origin, const Eigen::Vector2f &direction,
                        float max_range) const {
    float direction_norm = direction.norm();
    if (direction_norm <= 0.0f) {
        return max_range;
    }

    Eigen::Vector2f direction_unit = direction / direction_norm;

    float nearest = max_range;

    for (size_t i = 0; i < this->num_wall_segments; i++) {
        const FieldWallSegment &wall = this->wall_segments[i];

        Eigen::Vector2f wall_span = wall.end - wall.start;

        float denom = cross_2d(direction_unit, wall_span);
        if (fabsf(denom) <= 1e-6f) {
            continue;
        }

        Eigen::Vector2f origin_to_wall = wall.start - origin;

        float ray_distance = cross_2d(origin_to_wall, wall_span) / denom;
        float wall_alpha = cross_2d(origin_to_wall, direction_unit) / denom;

        if (ray_distance < 0.0f || wall_alpha < 0.0f || wall_alpha > 1.0f) {
            continue;
        }

        nearest = std::min(nearest, ray_distance);
    }

    return nearest;
}

size_t FieldMap::wall_count() const { return this->num_wall_segments; }
