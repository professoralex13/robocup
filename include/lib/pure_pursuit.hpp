#pragma once

#include "odometry.hpp"
#undef B1
#include "pid.hpp"
#include <Eigen/Geometry>
#include <optional>
#include <tuple>
#include <vector>

class PurePursuit {
  private:
    float look_ahead_distance;

    std::vector<Eigen::Vector2f> current_path;
    bool drive_path_backwards = false;

  public:
    PurePursuit(float look_ahead_distance);

    std::tuple<float, float> compute_errors(Pose current_pose);
    void set_current_path(std::vector<Eigen::Vector2f> positions, bool backwards = false);
    void set_drive_direction(bool backwards);
};