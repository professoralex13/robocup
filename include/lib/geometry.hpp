#include "lib/lidar_processing.hpp"
#include "lidar.hpp"

struct LineFit {
    float slope;
    float intercept;
};

LineFit line_fit(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, PointSpan range);