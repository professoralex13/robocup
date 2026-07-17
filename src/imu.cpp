#include "imu.hpp"
#include "telemetry_bus.hpp"

ImuTask::ImuTask(TwoWire *wire) : SchedulerTask("imu_task"), imu(55, 0x28, wire) {}

void ImuTask::setup() {
    if (!this->imu.begin()) {
        log_err("No IMU detected");
    }
}

void ImuTask::loop() {
    Eigen::Vector3f angles = this->get_euler_angles();
    telemetry::publish_f32(telemetry::KEY_PITCH, angles.x());
}

Eigen::Vector3f ImuTask::get_euler_angles() {
    imu::Vector<3> eulers = this->imu.getVector(Adafruit_BNO055::VECTOR_EULER);

    // Compass-style convention:
    // +heading is clockwise, 0 is forward, matching the robot/world visualization.
    return {-eulers.y() * DEG_TO_RAD, eulers.x() * DEG_TO_RAD, eulers.z() * DEG_TO_RAD};
}
