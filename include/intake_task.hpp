#pragma once

#include "lib/lidar.hpp"
#include "scheduler_task.hpp"
#include <HardwareSerial.h>
#include <HerkulexServo.h>
#include <etl/deque.h>

class IntakeTask : public SchedulerTask {
  private:
    HerkulexServoBus herkulexBus = HerkulexServoBus(Serial7);
    HerkulexServo servo_a = HerkulexServo(herkulexBus, 3);
    HerkulexServo servo_b = HerkulexServo(herkulexBus, 2);

    unsigned long last_update = 0;
    unsigned long now = 0;
    bool toggle = false;

  public:
    IntakeTask();

    void setup();
    void loop();

    int get_frequency() const override { return 60; }
};