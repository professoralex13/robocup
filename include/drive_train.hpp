#include "Encoder.h"
#include "scheduler_task.hpp"
#include <Servo.h>

class DriveTrainTask : public SchedulerTask {
  private:
    static const int LEFT_MOTOR_CONTROL_PIN = 0;
    static const int RIGHT_MOTOR_CONTROL_PIN = 1;

    static const int LEFT_MOTOR_ENCODER_PIN_A = 2;
    static const int LEFT_MOTOR_ENCODER_PIN_B = 3;
    static const int RIGHT_MOTOR_ENCODER_PIN_A = 4;
    static const int RIGHT_MOTOR_ENCODER_PIN_B = 5;

    static const int TICKS_PER_REVOLUTION = 2800;

    static constexpr float RADIANS_PER_TICK = 2.0 * PI / (float)TICKS_PER_REVOLUTION;

    Servo left_motor;
    Servo right_motor;

    uint32_t last_timestamp;

    int last_left_ticks = 0;
    int last_right_ticks = 0;

    Encoder left_encoder = Encoder(LEFT_MOTOR_ENCODER_PIN_A, LEFT_MOTOR_ENCODER_PIN_B);
    Encoder right_encoder = Encoder(RIGHT_MOTOR_ENCODER_PIN_A, RIGHT_MOTOR_ENCODER_PIN_B);

    float left_command = 0.0;
    float right_command = 0.0;

    float left_velocity = 0.0;
    float right_velocity = 0.0;

  public:
    DriveTrainTask();

    void setup();
    void loop();

    void set_commands(float left, float right);

    float get_left_wheel_velocity();
    float get_right_wheel_velocity();

    int get_frequency() const override { return 60; }
};