#include "tasks/mapping.hpp"

MappingTask::MappingTask(PositionTrackingTask *position_tracking_task, LidarTask *lidar_task)
    : SchedulerTask("mapping"), position_tracking_task(position_tracking_task),
      lidar_task(lidar_task) {}

void MappingTask::setup() { this->occupancy_grid.clear(); }

void MappingTask::loop() {
    Pose pose = this->position_tracking_task->get_current_pose();
    this->occupancy_grid.update_from_lidar(pose, this->lidar_task->get_points());
}

const OccupancyGridMap &MappingTask::get_occupancy_grid() const { return this->occupancy_grid; }
