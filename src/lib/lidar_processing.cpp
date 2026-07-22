#include "lib/lidar_processing.hpp"

bool coarse_cluster_check(LidarResponsePoint point1, LidarResponsePoint point2) {
    float threshold = std::min(point1.range, point2.range) * COARSE_THRESHOLD_RANGE_MULTIPLIER +
                      COARSE_THRESHOLD_OFFSET;

    return (point1.position - point2.position).norm() < threshold;
}

#include "Arduino.h"

std::vector<std::vector<LidarResponsePoint>>
get_coarse_clusters(std::span<LidarResponsePoint> points) {
    std::vector<std::vector<LidarResponsePoint>> clusters;

    if (points.empty()) {
        return clusters;
    }

    clusters.push_back({points[0]});

    for (int i = 1; i < points.size(); i++) {
        bool found = false;

        for (int j = clusters.size() - 1; j >= 0; j--) {
            if (coarse_cluster_check(points[i], clusters[j].back())) {
                clusters[j].push_back(points[i]);
                found = true;
                break;
            }
        }

        if (!found) {
            clusters.push_back({points[i]});
        }
    }

    auto first_cluster = clusters.front();
    auto last_cluster = clusters.back();

    // If the closest points on the start and end clusters are within the threshold, merge the
    // clusters
    if (clusters.size() >= 2 && coarse_cluster_check(first_cluster.front(), last_cluster.back())) {
        first_cluster.insert(first_cluster.end(), last_cluster.begin(), last_cluster.end());

        clusters.pop_back();
    }

    return clusters;
}

LidarProcessingResult LidarProcessing::process_points(std::span<LidarResponsePoint> points) {
    auto coarse_clusters = get_coarse_clusters(points);

    LidarProcessingResult result = {
        .coarse_segments = coarse_clusters,
    };

    return result;
}