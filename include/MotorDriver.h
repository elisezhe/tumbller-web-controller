#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

/// Low-level H-bridge motor driver (TB6612FNG).
/// Owns AIN1/BIN1/PWMA/PWMB/STBY pins – all motor I/O in one place.
class MotorDriver {
 public:
  void init();

  /// Zero both PWM outputs, set known direction state.
  void stop();

  /// Hardware enable / disable (STBY pin).
  void enable();
  void disable();

  /// Direct drive – bypasses PID (used for self-right maneuver).
  void forward(uint8_t speed);
  void back(uint8_t speed);

  /// Signed PWM: negative = reverse.  Called from balance ISR.
  void drive(int16_t left, int16_t right);
};

#endif
