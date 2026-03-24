#include "scheduler_task.hpp"
#include <HardwareSerial.h>

class LidarTask : public SchedulerTask {
  private:
    HardwareSerialIMXRT serial;

  public:
    LidarTask(HardwareSerialIMXRT serial);

    void setup();
    void loop();

    int get_frequency() const override { return 60; }
};