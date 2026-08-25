#include "lib/geometry.hpp"

LineFit line_fit(std::span<LidarResponsePoint> points) {
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

    for (auto &r : points) {
        auto &p = r.position;
        sum_x += p.x();
        sum_y += p.y();
        sum_xy += p.x() * p.y();
        sum_xx += p.x() * p.x();
    }

    int n = points.size();

    float denom = n * sum_xx - sum_x * sum_x;

    if (fabsf(denom) < 1e-6f)
        return {0.0f, sum_y / n}; // degenerate/near-vertical guard

    float slope = (n * sum_xy - sum_x * sum_y) / denom;
    return {slope, (sum_y - slope * sum_x) / n};
}