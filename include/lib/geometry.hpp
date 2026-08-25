#include "lib/lidar_processing.hpp"
#include "lidar.hpp"

struct LineFit {
    float slope;
    float intercept;
};

LineFit line_fit(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, PointSpan range);

struct CircleFit {
    float xc, yc, r;

    int worst_index;
    float worst_residual;

    float radius_deviation;
};

CircleFit circle_fit(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, PointSpan range);
