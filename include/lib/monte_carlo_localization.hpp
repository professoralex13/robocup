#pragma once

#include "lib/field_map.hpp"
#include "lib/lidar.hpp"
#include "lib/odometry.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <etl/deque.h>

class MonteCarloLocalization {
  public:
    static constexpr size_t NUM_PARTICLES = 128;
    static constexpr size_t LIDAR_HISTORY_CAPACITY = 1500;

    explicit MonteCarloLocalization(const FieldMap &field_map);

    void set_field_map(const FieldMap &field_map);

    // Initializes particles around a known or guessed robot pose.
    void set_initial_pose(const Pose &pose, float position_sigma, float heading_sigma);

    // Prediction step from odometry motion in robot frame.
    void predict(const Eigen::Vector2f &robot_travel, float heading_change);

    // Measurement step using lidar points and a beam model against the field map.
    void update_beam_model(const etl::deque<LidarResponsePoint, LIDAR_HISTORY_CAPACITY> &points);

    Pose get_estimated_pose() const;
    float get_position_uncertainty() const;

  private:
    struct Particle {
        Pose pose;
        float weight;
    };

    std::array<Particle, NUM_PARTICLES> particles;
    FieldMap field_map;
    Pose estimated_pose = {
        .position = Eigen::Vector2f::Zero(),
        .heading = 0.0f,
    };

    uint32_t rng_state = 0x9E3779B9;

    float position_noise_per_meter = 0.04f;
    float heading_noise_per_radian = 0.03f;
    float lidar_max_range = 12.0f;
    float lidar_min_range = 0.10f;
    float beam_sigma = 0.12f;

    float random_uniform();
    float random_symmetric();
    float random_gaussian();

    void normalize_weights();
    void low_variance_resample();
    void compute_estimated_pose();
};
