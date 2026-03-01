#include "PidController.h"

PidController::PidController(double kp, double ki, double kd,
                             double integral_limit)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), integral_limit_(integral_limit) {
}

double PidController::compute(double p_input, double i_input, double d_input) {
  integral_ += i_input;
  if (integral_limit_ > 0) {
    integral_ = constrain(integral_, -integral_limit_, integral_limit_);
  }
  return kp_ * p_input + ki_ * integral_ + kd_ * d_input;
}

void PidController::resetIntegral() {
  integral_ = 0;
}
