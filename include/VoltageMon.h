#ifndef VOLTAGEMON_H
#define VOLTAGEMON_H

#include <Arduino.h>

#include "mode.h"

class StatusLeds;

class VoltageMon {
 public:
  VoltageMon(CarState& state);
  void init(StatusLeds& leds);
  void measure();

 private:
  CarState& state_;
  StatusLeds* leds_;
  int low_voltage_flag_;
  unsigned long vol_measure_time_;
};

#endif
