#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>

/// Lightweight PID controller for real-time control loops.
///
/// The three-input compute() allows the proportional, integral, and
/// derivative terms to receive *different* signals.  Common patterns:
///
///   Standard PID   – compute(error, error, d_error)
///   P-on-measurement – compute(-measurement, error)   (avoids setpoint kick)
///   Feedforward + D  – compute(setpoint,    0, gyro)
class PidController {
 public:
  PidController(float kp, float ki, float kd, float integral_limit = 0);

  /// Compute PID output.
  /// @param p_input  Signal multiplied by Kp.
  /// @param i_input  Signal accumulated into the integral and multiplied by Ki.
  /// @param d_input  Signal multiplied by Kd.
  /// @param dt       Timestep scaling for the integral term (seconds).
  ///                 Default 1.0 preserves fixed-rate behavior.
  float compute(float p_input, float i_input, float d_input = 0, float dt = 1.0);

  void resetIntegral();
  float integral() const {
    return integral_;
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_;
  float integral_limit_;
};

#endif
