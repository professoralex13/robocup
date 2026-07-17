#include "scheduler_task.hpp"
#include <config.hpp>

class TelemetryTask : public SchedulerTask {
  public:
    TelemetryTask();

    void setup();
    void loop();

    int get_frequency() const override { return TELEMETRY_TASK_FREQ; }
};