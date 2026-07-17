#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <etl/deque.h>

#define LIDAR_POINTS_HISTORY 312

class LidarTask : public SchedulerTask {
  private:
    LidarDataReader reader;

    Eigen::Vector2f mount_position_in_robot_frame = Eigen::Vector2f::Zero();
    float mount_yaw_in_robot_frame = 0.0f;

    bool ignore_angle_enabled = false;
    float ignore_angle_start_radians = 0.0f;
    float ignore_angle_end_radians = 0.0f;

    LidarResponsePoint to_robot_frame(const LidarResponsePoint &point) const;
    bool is_angle_ignored(float angle_radians) const;

  public:
    LidarTask();

    void set_mount_pose_in_robot_frame(const Eigen::Vector2f &position, float yaw);
    void set_ignored_angle_range(float start_radians, float end_radians);
    void disable_ignored_angle_range();

    void setup();
    void loop();

    int get_frequency() const override { return 500; }

    etl::deque<LidarResponsePoint, LIDAR_POINTS_HISTORY> points;
};