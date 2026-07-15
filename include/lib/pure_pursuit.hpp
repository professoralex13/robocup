#include "eigen.h"
#include "odometry.hpp"
#include <optional>
#include <tuple>
#include <vector>

class PurePursuit {
  private:
    float look_ahead_distance;

    std::vector<Eigen::Vector2f> current_path;
    bool drive_path_backwards = false;

    float remaining_distance = 0.0;

    std::tuple<float, float> compute_errors(Pose current_pose);

  public:
    PurePursuit(float look_ahead_distance);

    void set_current_path(std::vector<Eigen::Vector2f> positions, bool backwards = false);
    void set_drive_direction(bool backwards);
};