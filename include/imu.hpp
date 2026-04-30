#include "eigen.h"
#include "scheduler_task.hpp"
#include <Adafruit_BNO055.h>
#include <HardwareSerial.h>
#include <Wire.h>

class ImuTask : public SchedulerTask {
  private:
    Adafruit_BNO055 imu;

  public:
    ImuTask(TwoWire *port);

    void setup();
    void loop();

    Eigen::Vector3f get_euler_angles();

    int get_frequency() const override { return 5; }
};