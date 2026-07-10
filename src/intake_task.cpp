#include "intake_task.hpp"
#include "Arduino.h"

IntakeTask::IntakeTask() : SchedulerTask("intake_task") {}

void IntakeTask::setup() {
    Serial7.begin(115200);

    servo_a.setTorqueOn();
    servo_b.setTorqueOn();
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
            servo_a.setPosition(angleToNum(0.0), 50, HerkulexLed::Green);
            servo_b.setPosition(angleToNum(0.0), 50, HerkulexLed::Green);
        } else {
            // move to +90° over a duration of 560ms, set LED to blue
            // 512 + 90°/0.325 = 789
            servo_a.setPosition(angleToNum(90.0), 50, HerkulexLed::Blue);
            servo_b.setPosition(angleToNum(90.0), 50, HerkulexLed::Blue);
        }

        last_update = now;
        toggle = !toggle;
    }
}
