#include "tasks/intake.hpp"
#include "Arduino.h"

IntakeTask::IntakeTask() : SchedulerTask("intake_task") {}

void IntakeTask::setup() {
    Serial7.begin(115200);

    left_servo.setTorqueOn();
    right_servo.setTorqueOn();
}

int angleToNum(float angle) { return 512 + (int)(angle / 0.325); }

void IntakeTask::loop() {
    herkulexBus.update();

    now = millis();

    if ((now - last_update) > 2000) {
        // called every 1000 ms
        if (toggle) {
            // move to -90° over a duration of 560ms, set LED to green
            // 512 - 90°/0.325 = 235
            left_servo.setPosition(angleToNum(-15.0), 100, HerkulexLed::Green);
            right_servo.setPosition(angleToNum(9.0), 100, HerkulexLed::Green);
        } else {
            // move to +90° over a duration of 560ms, set LED to blue
            // 512 + 90°/0.325 = 789
            left_servo.setPosition(angleToNum(45.0), 100, HerkulexLed::Blue);
            right_servo.setPosition(angleToNum(-51.0), 100, HerkulexLed::Blue);
        } // 512 + 90°/0.325 = 789

        last_update = now;
        toggle = !toggle;
    }
}
