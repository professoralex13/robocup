#pragma once

#include "config.hpp"
#include "etl/vector.h"
#include "lib/lidar.hpp"

// Worst case every point is its own cluster, so cap at MAX_POINTS
using ClusterList = etl::vector<std::span<LidarResponsePoint>, MAX_LIDAR_POINTS>;

struct LidarProcessingResult {
    etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points;
    ClusterList line_segments;
};

class LidarProcessing {
  public:
    LidarProcessingResult process_points(std::span<LidarResponsePoint> points);
};