#ifndef VOLTAGE_H
#define VOLTAGE_H

#include <Arduino.h>

#include "mode.h"

class RGB;

class VoltageMon {
 public:
  VoltageMon(CarState& state);
  void init(RGB& rgb);
  void measure();

 private:
  CarState& state_;
  RGB* rgb_;
  int low_voltage_flag_;
  unsigned long vol_measure_time_;
};

#endif
