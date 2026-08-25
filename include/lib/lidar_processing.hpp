#pragma once

#include "config.hpp"
#include "etl/vector.h"
#include "lib/lidar.hpp"

struct PointSpan {
    size_t start;
    size_t count;
};

// Worst case every point is its own cluster, so cap at MAX_POINTS
using ClusterList = etl::vector<PointSpan, MAX_LIDAR_POINTS>;

struct LidarProcessingResult {
    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    ClusterList line_segments;
};

class LidarProcessing {
  public:
    LidarProcessingResult process_points(std::span<LidarResponsePoint> points);
};