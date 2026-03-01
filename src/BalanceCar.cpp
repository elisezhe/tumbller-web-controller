#include "BalanceCar.h"

#include "I2Cdev.h"
#include "MotorDriver.h"
#include "MsTimer2.h"
#include "Pins.h"
#include "Wire.h"

BalanceCar* BalanceCar::instance_ = nullptr;

BalanceCar::BalanceCar(CarState& state)
    : kalmanfilter(),
      kalmanfilter_angle(0),
      balance_angle_min(-22),
      balance_angle_max(22),
      setting_car_speed(0),
      setting_turn_speed(0),
      pid_balance(55, 0, 0.75),
      pid_speed(10, 6.5, 0, 120),
      pid_turn(1, 0, 0.5),
      suppress_motor_output(false),
      state_(state),
      motors_(nullptr),
      mpu_(),
      angle_zero_(0),
      angular_velocity_zero_(0),
      encoder_left_(),
      encoder_right_(),
      ax_(0),
      ay_(0),
      az_(0),
      gx_(0),
      gy_(0),
      gz_(0),
      last_micros_(0),
      speed_dt_(0),
      speed_control_output_(0),
      speed_filter_(0),
      speed_control_period_count_(0),
      speed_filter_old_(0),
      pwm_left_(0),
      pwm_right_(0) {
}

void BalanceCar::init(MotorDriver& motors) {
  instance_ = this;
  motors_ = &motors;

  Wire.begin();
  mpu_.initialize();
  kalmanfilter.init(0.005, 0.001, 0.005, 0.5, 1, 128.1);

  encoder_left_.init(ENCODER_LEFT_A_PIN, true);
  encoder_right_.init(ENCODER_RIGHT_A_PIN, false);
  last_micros_ = micros();
  MsTimer2::set(5, balanceISR);
  MsTimer2::start();
}

void BalanceCar::balanceISR() {
  if (instance_)
    instance_->balance();
}

void BalanceCar::balance() {
  sei();

  // Measure actual elapsed time
  unsigned long now = micros();
  float dt = (now - last_micros_) * 1e-6f;
  last_micros_ = now;
  if (dt <= 0 || dt > 0.02f)
    dt = 0.005f;  // clamp on first call / overflow

  encoder_left_.update(pwm_left_);
  encoder_right_.update(pwm_right_);

  mpu_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
  kalmanfilter.process(ax_, ay_, az_, gx_, gy_, gz_, dt);
  kalmanfilter_angle = kalmanfilter.angle;

  // Balance PID (Ki=0, dt has no effect on integral)
  double balance_control_output =
      pid_balance.compute(kalmanfilter_angle - angle_zero_, 0,
                          kalmanfilter.gyro_x - angular_velocity_zero_);

  // Speed PID (every 8 cycles, ~40ms)
  speed_dt_ += dt;
  speed_control_period_count_++;
  if (speed_control_period_count_ >= 8) {
    speed_control_period_count_ = 0;
    double car_speed =
        (encoder_left_.readAndReset() + encoder_right_.readAndReset()) * 0.5;
    speed_filter_ = speed_filter_old_ * 0.7 + car_speed * 0.3;
    speed_filter_old_ = speed_filter_;
    double speed_error = setting_car_speed - speed_filter_;
    speed_control_output_ =
        pid_speed.compute(-speed_filter_, speed_error, 0, speed_dt_);
    speed_dt_ = 0;
  }

  // Turn control (Ki=0, dt has no effect on integral)
  double rotation_control_output =
      pid_turn.compute(setting_turn_speed, 0, kalmanfilter.gyro_z);

  // Compute PWM
  pwm_left_ =
      balance_control_output - speed_control_output_ - rotation_control_output;
  pwm_right_ =
      balance_control_output - speed_control_output_ + rotation_control_output;
  pwm_left_ = constrain(pwm_left_, -255, 255);
  pwm_right_ = constrain(pwm_right_, -255, 255);

  // Fall detection
  if (state_.motion_mode != START && state_.motion_mode != STOP &&
      (kalmanfilter_angle < balance_angle_min ||
       balance_angle_max < kalmanfilter_angle)) {
    state_.motion_mode = STOP;
    motors_->stop();
  }

  // Motor output
  if (state_.motion_mode == STOP) {
    pid_speed.resetIntegral();
    setting_car_speed = 0;
    pwm_left_ = 0;
    pwm_right_ = 0;
    if (!suppress_motor_output) {
      motors_->stop();
    }
  } else {
    motors_->drive(pwm_left_, pwm_right_);
  }
}
