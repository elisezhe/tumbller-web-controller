#ifndef WHEELENCODER_H
#define WHEELENCODER_H

#include <Arduino.h>

/// Reads a single quadrature encoder channel via pin-change interrupt.
/// Accumulates signed pulses (direction supplied externally from PWM sign).
class WheelEncoder {
 public:
  WheelEncoder();

  /// Attach the pin-change interrupt.  Call once during setup().
  /// @param pin         The digital pin connected to encoder channel A.
  /// @param use_pcint   True to use PINCHANGEINTERRUPT (needed for pin 2).
  void init(uint8_t pin, bool use_pcint = false);

  /// Called every control cycle: accumulates raw ticks into signed distance,
  /// using the sign of the current PWM to determine direction, then resets
  /// the raw tick counter.
  /// @param pwm  Current PWM value (sign gives direction).
  void update(float pwm);

  /// Return accumulated signed pulses since the last reset, then reset.
  int readAndReset();

 private:
  // PORT(RPi): replace with std::atomic<uint16_t> and remove
  // noInterrupts()/interrupts() guard in update().
  /// reset every 5ms — small count, but 16-bit for safety
  volatile uint16_t ticks_;
  int accumulated_;

  /// Per-instance ISR dispatch.
  void tick();

  // Static ISR trampoline table (supports up to 2 encoders).
  static WheelEncoder* instances_[2];
  static uint8_t instance_count_;
  static void isr0();
  static void isr1();
};

#endif
