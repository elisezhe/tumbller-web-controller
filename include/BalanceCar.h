#ifndef BALANCECAR_H
#define BALANCECAR_H

#include <Arduino.h>

#include "KalmanFilter.h"
#include "MPU6050.h"
#include "PidController.h"
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
  int setting_car_speed;
  int setting_turn_speed;

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

  double angle_zero_;
  double angular_velocity_zero_;

  volatile unsigned long encoder_count_right_a_;
  volatile unsigned long encoder_count_left_a_;
  int16_t ax_, ay_, az_, gx_, gy_, gz_;

  int encoder_left_pulse_num_speed_;
  int encoder_right_pulse_num_speed_;
  double speed_control_output_;
  double speed_filter_;
  int speed_control_period_count_;
  double speed_filter_old_;
  double pwm_left_;
  double pwm_right_;

  void balance();

  static BalanceCar* instance_;
  static void balanceISR();
  static void encoderLeftISR();
  static void encoderRightISR();
};

#endif
