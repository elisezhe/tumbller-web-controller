#ifndef BALANCECAR_H
#define BALANCECAR_H

#include <Arduino.h>

#include "KalmanFilter.h"
#include "MPU6050.h"
#include "PidController.h"
#include "WheelEncoder.h"
#include "mode.h"

class MotorDriver;

class BalanceCar {
 public:
  BalanceCar(CarState& state);
  void init(MotorDriver& motors);

  KalmanFilter kalmanfilter;
  float kalmanfilter_angle;
  char balance_angle_min;
  char balance_angle_max;
  volatile int8_t setting_car_speed;   // written from loop(), read from ISR
  volatile int8_t setting_turn_speed;  // written from loop(), read from ISR

  PidController pid_balance;
  PidController pid_speed;
  PidController pid_turn;

  /// Set true while directly driving motors (e.g. self-right).
  /// The balance ISR will skip motor output while this is set.
  bool suppress_motor_output;

 private:
  CarState& state_;
  MotorDriver* motors_;
  MPU6050 mpu_;

  float angle_zero_;
  float angular_velocity_zero_;

  WheelEncoder encoder_left_;
  WheelEncoder encoder_right_;
  int16_t ax_, ay_, az_, gx_, gy_, gz_;

  unsigned long last_micros_;
  float speed_dt_;
  float speed_control_output_;
  float speed_filter_;
  int speed_control_period_count_;
  float pwm_left_;
  float pwm_right_;

  void balance();

  static BalanceCar* instance_;
  static void balanceISR();
};

#endif
