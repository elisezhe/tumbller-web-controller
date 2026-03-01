#ifndef VOLTAGEMON_H
#define VOLTAGEMON_H

#include <Arduino.h>

enum VoltageLevel : uint8_t { VOLTAGE_OK, VOLTAGE_LOW, VOLTAGE_CRITICAL };

/// Pure voltage sensor — reads battery ADC, reports level.
/// No actuation, no LED control, no state mutation.
class VoltageMon {
 public:
  VoltageMon();
  void init();

  /// Read ADC every 1000ms, update cached level.
  void update();

  /// Current cached voltage level.
  VoltageLevel level() const {
    return level_;
  }

 private:
  VoltageLevel level_;
  unsigned long measure_time_;
};

#endif
