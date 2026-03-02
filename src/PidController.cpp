#include "PidController.h"

PidController::PidController(float kp, float ki, float kd, float integral_limit)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), integral_limit_(integral_limit) {
}

float PidController::compute(float p_input, float i_input, float d_input, float dt) {
  integral_ += i_input * dt;
  if (integral_limit_ > 0) {
    integral_ = constrain(integral_, -integral_limit_, integral_limit_);
  }
  return kp_ * p_input + ki_ * integral_ + kd_ * d_input;
}

void PidController::resetIntegral() {
  integral_ = 0;
}
