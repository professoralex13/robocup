#include "lib/lidar_processing.hpp"
#include "etl/vector.h"
#include <Eigen/QR>

bool coarse_cluster_check(LidarResponsePoint point1, LidarResponsePoint point2) {
    float threshold = std::min(point1.range, point2.range) * COARSE_THRESHOLD_RANGE_MULTIPLIER +
                      COARSE_THRESHOLD_OFFSET;

    return (point1.position - point2.position).norm() < threshold;
}

ClusterList get_coarse_clusters(std::span<LidarResponsePoint> points) {
    ClusterList clusters;

    if (points.empty()) {
        return clusters;
    }

    clusters.push_back({0, 1});

    for (uint16_t i = 1; i < points.size(); i++) {
        bool found = false;

        for (int j = clusters.size() - 1; j >= 0; j--) {
            uint16_t last_idx = clusters[j].start + clusters[j].count - 1;

            if (coarse_cluster_check(points[i], points[last_idx])) {
                clusters[j].count++;
                found = true;
                break;
            }
        }

        if (!found) {
            clusters.push_back({i, 1});
        }
    }

    if (clusters.size() >= 2) {
        Range first = clusters.front();
        Range last = clusters.back();

        if (coarse_cluster_check(points[first.start], points[last.start + last.count - 1])) {
            std::rotate(points.begin(), points.begin() + first.count, points.end());

            Range merged{last.start - first.count, last.count + first.count};

            ClusterList updated;
            updated.push_back(merged);

            for (size_t k = 1; k + 1 < clusters.size(); k++) {
                updated.push_back({clusters[k].start - first.count, clusters[k].count});
            }

            clusters = updated;
        }
    }

    return clusters;
}

struct LineFit {
    float slope;
    float intercept;
};

LineFit line_fit(std::span<LidarResponsePoint> points, Range range) {
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

    int n = range.count;

    for (int k = 0; k < range.count; k++) {
        auto &p = points[range.start + k].position;
        sum_x += p.x();
        sum_y += p.y();
        sum_xy += p.x() * p.y();
        sum_xx += p.x() * p.x();
    }

    float denom = n * sum_xx - sum_x * sum_x;
    if (fabsf(denom) < 1e-6f)
        return {0.0f, sum_y / n}; // degenerate/near-vertical guard

    float slope = (n * sum_xy - sum_x * sum_y) / denom;
    return {slope, (sum_y - slope * sum_x) / n};
}

const float LINE_NOISE_THRESHOLD = 0.01f;

ClusterList fit_clusters(std::span<LidarResponsePoint> points, const ClusterList &coarse_clusters) {
    ClusterList work_stack;
    ClusterList line_segments;

    for (auto &r : coarse_clusters) {
        if (r.count >= 2) {
            work_stack.push_back(r);
        }
    }

    while (!work_stack.empty()) {
        Range range = work_stack.back();

        work_stack.pop_back();

        auto [slope, intercept] = line_fit(points, range);
        float norm = sqrtf(slope * slope + 1);
        float furthest_distance = 0.0f;
        int furthest_offset = -1;

        for (int k = 0; k < range.count; k++) {
            auto &p = points[range.start + k].position;

            float distance = fabsf(slope * p.x() - p.y() + intercept) / norm;

            if (distance > furthest_distance) {
                furthest_distance = distance;
                furthest_offset = k;
            }
        }

        if (furthest_distance > LINE_NOISE_THRESHOLD) {
            Range first{range.start, furthest_offset + 1};
            Range second{range.start + furthest_offset + 1, range.count - furthest_offset - 1};

            if (first.count == range.count) {
                first.count = range.count - 1;
            }

            if (second.count == range.count) {
                second.start = second.start + 1;
                second.count = second.count - 1;
            }

            if (first.count >= 2) {
                work_stack.push_back(first);
            }

            if (second.count >= 2) {
                work_stack.push_back(second);
            }
        } else {
            line_segments.push_back(range);
        }
    }

    return line_segments;
}

LidarProcessingResult
LidarProcessing::process_points(std::span<LidarResponsePoint> unsorted_points) {
    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points(unsorted_points.begin(),
                                                             unsorted_points.end());

    std::sort(points.begin(), points.end(), [](auto a, auto b) { return a.angle < b.angle; });

    ClusterList coarse_clusters = get_coarse_clusters(points);
    ClusterList line_segments = fit_clusters(points, coarse_clusters);

    LidarProcessingResult result;

    result.points = points;
    result.line_segments = line_segments;

    return result;
}