#include "lib/geometry.hpp"

LineFit line_fit(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, PointSpan range) {
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

    for (int i = range.start; i < range.start + range.count; i++) {
        auto &p = points[i].position;

        sum_x += p.x();
        sum_y += p.y();
        sum_xy += p.x() * p.y();
        sum_xx += p.x() * p.x();
    }

    int n = range.count;

    float denom = n * sum_xx - sum_x * sum_x;

    if (fabsf(denom) < 1e-6f)
        return {0.0f, sum_y / n}; // degenerate/near-vertical guard

    float slope = (n * sum_xy - sum_x * sum_y) / denom;
    return {slope, (sum_y - slope * sum_x) / n};
}