#pragma once

#include "eigen.h"
#include <array>
#include <cstddef>

struct FieldWallSegment {
    Eigen::Vector2f start;
    Eigen::Vector2f end;
};

class FieldMap {
  public:
    static constexpr size_t MAX_WALL_SEGMENTS = 16;

    static FieldMap make_square(float side_length_meters);
    static FieldMap make_rectangle(float width_x_meters, float height_y_meters);

    bool add_wall(const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    void clear();

    float raycast(const Eigen::Vector2f &origin, const Eigen::Vector2f &direction,
                  float max_range) const;

    size_t wall_count() const;

  private:
    std::array<FieldWallSegment, MAX_WALL_SEGMENTS> wall_segments;
    size_t num_wall_segments = 0;
};
