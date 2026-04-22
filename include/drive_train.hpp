#include "scheduler_task.hpp"
#include <Servo.h>

class DriveTrainTask : public SchedulerTask {
  private:
    Servo left_motor;
    Servo right_motor;

    int left_port;
    int right_port;

    float left_command = 0.0;
    float right_command = 0.0;

  public:
    DriveTrainTask(int left_port, int right_port);

    void setup();
    void loop();

    void set_commands(float left, float right);

    int get_frequency() const override { return 60; }
};