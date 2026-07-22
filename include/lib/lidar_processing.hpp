#include "config.hpp"
#include "etl/vector.h"
#include "lib/lidar.hpp"

struct LidarProcessingResult {
    std::vector<std::vector<LidarResponsePoint>> coarse_segments;
};

class LidarProcessing {
  public:
    LidarProcessingResult process_points(std::span<LidarResponsePoint> points);
};