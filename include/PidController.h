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
  PidController(double kp, double ki, double kd, double integral_limit = 0);

  /// Compute PID output.
  /// @param p_input  Signal multiplied by Kp.
  /// @param i_input  Signal accumulated into the integral and multiplied by Ki.
  /// @param d_input  Signal multiplied by Kd.
  double compute(double p_input, double i_input, double d_input = 0);

  void resetIntegral();
  double integral() const {
    return integral_;
  }

 private:
  double kp_;
  double ki_;
  double kd_;
  double integral_;
  double integral_limit_;
};

#endif
