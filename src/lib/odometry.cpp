#include "lib/odometry.hpp"
#include <Arduino.h>
#undef B1
#include <Eigen/Geometry>
#include <Eigen/QR>

OdometryModule::OdometryModule(const std::array<TrackingWheel, NUM_WHEELS> &tracking_wheels)
    : tracking_wheels(tracking_wheels) {
    Eigen::Matrix<float, 2, NUM_WHEELS> d;

    for (int i = 0; i < NUM_WHEELS; i++) {
        d.row(i) = tracking_wheels[i].direction;
    }

    this->weighting_matrix = d.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::Vector2f OdometryModule::compute_travel(float wheel_travels[NUM_WHEELS],
                                               float angle_change) {
    float actual_travels[NUM_WHEELS];

    for (int i = 0; i < NUM_WHEELS; i++) {
        TrackingWheel wheel = this->tracking_wheels[i];

        Eigen::Vector2f measured_travel = wheel.direction * wheel_travels[i];

        Eigen::Vector2f rotation_travel = wheel.location.unitOrthogonal() * angle_change;

        if (measured_travel.squaredNorm() == 0.0) {
            actual_travels[i] = 0.0;
        } else {
            Eigen::Vector2f actual_travel =
                measured_travel *
                (1.0 - (rotation_travel.dot(measured_travel) / measured_travel.squaredNorm()));

            actual_travels[i] = actual_travel.dot(wheel.direction);
        }
    }

    Eigen::Matrix2f bend_matrix;

    if (angle_change == 0.0) {
        bend_matrix << 0.0, 0.0,
            /*      */ 0.0, 0.0;
    } else {
        float sin_coef = sinf(angle_change) / angle_change;
        float cos_coef = (1.0 - cosf(angle_change)) / angle_change;

        bend_matrix << sin_coef, -cos_coef,
            /*      */ cos_coef, sin_coef;
    }

    return bend_matrix *
           (this->weighting_matrix * Eigen::Vector<float, NUM_WHEELS>(actual_travels));
}