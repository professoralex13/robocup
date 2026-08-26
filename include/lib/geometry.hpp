#pragma once

#include "lib/lidar_processing.hpp"
#include "lidar.hpp"

LineFit fit_line(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, PointSpan range);

CircleFit fit_circle(etl::vector<LidarResponsePoint, MAX_LIDAR_POINTS> points, PointSpan range);
