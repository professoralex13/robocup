#include "imu.hpp"

ImuTask::ImuTask(TwoWire *wire) : SchedulerTask("imu_task"), imu(55, 0x28, wire) {}

void ImuTask::setup() {
    if (!this->imu.begin()) {
        log_err("No IMU detected");
    }
}

void ImuTask::loop() {
    imu::Vector<3> eulers = this->imu.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> lin_accel = this->imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}
