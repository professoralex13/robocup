#include "lib/occupancy_grid_map.hpp"
#include <algorithm>
#include <cmath>

namespace {

int abs_int(int value) { return value < 0 ? -value : value; }

} // namespace

OccupancyGridMap::OccupancyGridMap() { this->clear(); }

void OccupancyGridMap::clear(uint8_t score) { this->scores.fill(score); }

void OccupancyGridMap::update_from_lidar(const Pose &robot_pose,
                                         std::span<const LidarResponsePoint> points) {
    float heading = robot_pose.heading;
    float heading_sin = sinf(heading);
    float heading_cos = cosf(heading);

    for (const LidarResponsePoint &point : points) {
        if (point.range <= 0.01f) {
            continue;
        }

        Eigen::Vector2f world_direction(
            heading_cos * point.position.x() + heading_sin * point.position.y(),
            -heading_sin * point.position.x() + heading_cos * point.position.y());

        Eigen::Vector2f hit_world = robot_pose.position + world_direction;

        this->apply_beam(robot_pose.position, hit_world);
    }
}

uint8_t OccupancyGridMap::get_score(size_t x, size_t y) const {
    if (x >= GRID_WIDTH || y >= GRID_HEIGHT) {
        return UNKNOWN_SCORE;
    }

    return this->scores[to_index(x, y)];
}

const std::array<uint8_t, OccupancyGridMap::GRID_WIDTH * OccupancyGridMap::GRID_HEIGHT> &
OccupancyGridMap::get_scores() const {
    return this->scores;
}

size_t OccupancyGridMap::width() const { return GRID_WIDTH; }

size_t OccupancyGridMap::height() const { return GRID_HEIGHT; }

float OccupancyGridMap::tile_size_meters() const { return TILE_SIZE_METERS; }

size_t OccupancyGridMap::to_index(size_t x, size_t y) { return y * GRID_WIDTH + x; }

bool OccupancyGridMap::world_to_grid(const Eigen::Vector2f &position, int &grid_x,
                                     int &grid_y) const {
    int x = (int)floorf(position.x() / TILE_SIZE_METERS);
    int y = (int)floorf(position.y() / TILE_SIZE_METERS);

    if (x < 0 || y < 0 || x >= (int)GRID_WIDTH || y >= (int)GRID_HEIGHT) {
        return false;
    }

    grid_x = x;
    grid_y = y;
    return true;
}

void OccupancyGridMap::apply_beam(const Eigen::Vector2f &origin_world,
                                  const Eigen::Vector2f &hit_world) {
    int origin_x = 0;
    int origin_y = 0;
    int hit_x = 0;
    int hit_y = 0;

    if (!this->world_to_grid(origin_world, origin_x, origin_y)) {
        return;
    }

    if (!this->world_to_grid(hit_world, hit_x, hit_y)) {
        return;
    }

    int x = origin_x;
    int y = origin_y;

    int dx = abs_int(hit_x - origin_x);
    int sx = origin_x < hit_x ? 1 : -1;
    int dy = -abs_int(hit_y - origin_y);
    int sy = origin_y < hit_y ? 1 : -1;
    int error = dx + dy;

    while (true) {
        if (x == hit_x && y == hit_y) {
            this->increase_cell(x, y);
            break;
        }

        this->decrease_cell(x, y);

        int e2 = 2 * error;

        if (e2 >= dy) {
            error += dy;
            x += sx;
        }

        if (e2 <= dx) {
            error += dx;
            y += sy;
        }
    }
}

void OccupancyGridMap::increase_cell(int grid_x, int grid_y) {
    size_t idx = to_index((size_t)grid_x, (size_t)grid_y);
    uint16_t increased = (uint16_t)this->scores[idx] + OCCUPIED_INCREMENT;
    this->scores[idx] = (uint8_t)std::min<uint16_t>(255, increased);
}

void OccupancyGridMap::decrease_cell(int grid_x, int grid_y) {
    size_t idx = to_index((size_t)grid_x, (size_t)grid_y);
    uint8_t current = this->scores[idx];

    if (current <= FREE_DECREMENT) {
        this->scores[idx] = 0;
        return;
    }

    this->scores[idx] = (uint8_t)(current - FREE_DECREMENT);
}
