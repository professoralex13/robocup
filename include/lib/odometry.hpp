#pragma once

#include "eigen.h"

struct Pose {
    Eigen::Vector2f position;
    float heading;
};

struct TrackingWheel {
    Eigen::Vector2f location;
    Eigen::Vector2f direction;
};

#define NUM_WHEELS 2

class OdometryModule {
  private:
    std::array<TrackingWheel, NUM_WHEELS> tracking_wheels;

  public:
    OdometryModule(const std::array<TrackingWheel, NUM_WHEELS> &tracking_wheels);

    Eigen::Matrix<float, NUM_WHEELS, 2> weighting_matrix;
    Eigen::Vector2f compute_travel(float wheel_travels[NUM_WHEELS], float angle_change);
};
