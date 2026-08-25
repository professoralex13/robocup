#include "lidar.hpp"

struct LineFit {
    float slope;
    float intercept;
};

LineFit line_fit(std::span<LidarResponsePoint> points);