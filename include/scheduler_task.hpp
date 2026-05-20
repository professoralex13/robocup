#pragma once

class SchedulerTask {
  protected:
    SchedulerTask(const char *name);

  public:
    const char *task_name;
    virtual void setup() = 0;
    virtual void loop() = 0;
    virtual int get_frequency() const = 0;

    int get_period_micros() const { return 1e6 / this->get_frequency(); };

    void log(const char *format, ...);
    void log_err(const char *format, ...);
};