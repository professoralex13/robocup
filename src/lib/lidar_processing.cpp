#include "lib/lidar_processing.hpp"
#include "etl/vector.h"
#include <Eigen/QR>
#include <lib/geometry.hpp>

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

    size_t start = 0, count = 1;

    for (size_t i = 1; i < points.size(); i++) {
        if (coarse_cluster_check(points[i], points[start + count - 1])) {
            count++;
        } else {
            clusters.push_back({start, count});
            start = i;
            count = 1;
        }
    }

    clusters.push_back({start, count});

    // TODO: This will need fixing

    // if (clusters.size() >= 2) {
    //     auto first = clusters.front();
    //     auto last = clusters.back();

    //     if (coarse_cluster_check(points[first.start], points[last.start + last.count - 1])) {
    //         std::rotate(points.begin(), points.begin() + first.count, points.end());

    //         // Every span computed before the rotate is now stale — must rebuild.
    //         ClusterList updated = {{.start = points.size() - last.count - first.count,
    //                                 .count = last.count + first.count}};

    //         size_t offset = updated[0].count;

    //         for (size_t k = 1; k + 1 < clusters.size(); k++) {
    //             updated.push_back({offset, clusters[k].count});
    //             offset += clusters[k].count;
    //         }

    //         clusters = updated;
    //     }
    // }

    return clusters;
}

const float LINE_NOISE_THRESHOLD = 0.02f;

ClusterList fit_clusters(const ClusterList &coarse_clusters,
                         etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points) {
    ClusterList work_stack;
    ClusterList line_segments;

    for (auto s : coarse_clusters) {
        work_stack.push_back(s);
    }

    while (!work_stack.empty()) {
        auto range = work_stack.back();
        work_stack.pop_back();

        auto [slope, intercept] = line_fit(points, range);
        float norm = sqrtf(slope * slope + 1);
        float furthest_distance = 0.0f;

        size_t split = 0;

        for (int i = 0; i < range.count; i++) {
            auto p = points[range.start + i].position;

            float distance = fabsf(slope * p.x() - p.y() + intercept) / norm;

            if (distance > furthest_distance) {
                furthest_distance = distance;
                split = i;
            }
        }

        if (furthest_distance <= LINE_NOISE_THRESHOLD) {
            line_segments.push_back(range);
            continue;
        }

        PointSpan first, second;

        if (split == range.count - 1) {
            // Furthest point is the last point in range. subspan(0, split+1) would be
            // the whole range and subspan(split+1) would be empty — infinite loop.
            first = {range.start, split};
            second = {range.start + split, 0};
        } else {
            first = {range.start, split + 1};
            second = {range.start + split + 1, range.count - split + 1};
        }

        if (first.count >= 2) {
            work_stack.push_back(first);
        }

        if (second.count >= 2) {
            work_stack.push_back(second);
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
    ClusterList line_segments = fit_clusters(coarse_clusters, points);

    LidarProcessingResult result;

    result.line_segments = coarse_clusters;
    result.points = points;

    return result;
}