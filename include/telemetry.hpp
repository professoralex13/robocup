#include "scheduler_task.hpp"

class TelemetryTask : public SchedulerTask {
  public:
    TelemetryTask();

    void setup();
    void loop();

    int get_frequency() const override { return 10; }
};