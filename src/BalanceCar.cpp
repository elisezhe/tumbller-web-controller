#include "BalanceCar.h"

#include "I2Cdev.h"
#include "MotorDriver.h"
#include "MsTimer2.h"
#include "Pins.h"
#include "Wire.h"

namespace {
// --- PID tuning gains ---
// Balance PD: proportional on angle error, derivative on gyro rate.
const float kBalanceKp = 55.0;
const float kBalanceKd = 0.75;

// Speed PI: P-on-measurement (avoids setpoint kick), I on error.
// Ki is time-scaled (seconds); equivalent to 0.26 at fixed 40ms update rate.
const float kSpeedKp = 10.0;
const float kSpeedKi = 6.5;
const float kSpeedIntegralLimit =
    120.0;  // max |integral| (780 max PWM contribution)

// Turn PD: feedforward on setpoint, derivative damping from yaw gyro.
const float kTurnKp = 1.0;
const float kTurnKd = 0.5;

// --- Balance / fall detection ---
const int8_t kFallAngleMin = -22;  // degrees — below this the robot is fallen
const int8_t kFallAngleMax = 22;   // degrees — above this the robot is fallen

// --- Speed control loop ---
const uint8_t kSpeedPidPeriod =
    8;  // ISR cycles between speed PID updates (~40ms)
const float kSpeedFilterAlpha = 0.3;  // EMA weight for new encoder sample

// --- PWM output ---
const int16_t kPwmLimit = 255;  // max absolute PWM value (8-bit timer)

// --- Timer interrupt ---
const uint16_t kBalanceIntervalMs = 5;  // MsTimer2 period

// --- Variable-dt safety clamp ---
const float kDtMax = 0.02f;  // 20ms — reject impossibly long intervals
const float kDtDefault =
    0.005f;  // 5ms — fallback for first call / micros() overflow

// --- Kalman filter init parameters ---
const float kKalmanDt = 0.005f;      // nominal timestep (seconds)
const float kKalmanQAngle = 0.001f;  // process noise covariance: angle state
const float kKalmanQGyro = 0.005f;   // process noise covariance: gyro bias
const float kKalmanRAngle = 0.5f;  // measurement noise covariance: accel angle
const float kKalmanC0 = 1.0f;      // observation model coefficient
const float kKalmanGyroOffset =
    128.1f;  // raw gyro X offset (hardware-specific)
}  // namespace

BalanceCar* BalanceCar::instance_ = nullptr;

BalanceCar::BalanceCar(CarState& state)
    : kalmanfilter(),
      kalmanfilter_angle(0),
      balance_angle_min(kFallAngleMin),
      balance_angle_max(kFallAngleMax),
      setting_car_speed(0),
      setting_turn_speed(0),
      pid_balance(kBalanceKp, 0, kBalanceKd),
      pid_speed(kSpeedKp, kSpeedKi, 0, kSpeedIntegralLimit),
      pid_turn(kTurnKp, 0, kTurnKd),
      suppress_motor_output(false),
      state_(state),
      motors_(nullptr),
      mpu_(),
      angle_zero_(0),
      angular_velocity_zero_(0),
      encoder_left_(),
      encoder_right_(),
      last_micros_(0),
      speed_dt_(0),
      speed_control_output_(0),
      speed_filter_(0),
      speed_control_period_count_(0),
      pwm_left_(0),
      pwm_right_(0) {
}

void BalanceCar::init(MotorDriver& motors) {
  instance_ = this;
  motors_ = &motors;

  Wire.begin();
  mpu_.initialize();
  kalmanfilter.init(kKalmanDt, kKalmanQAngle, kKalmanQGyro, kKalmanRAngle,
                    kKalmanC0, kKalmanGyroOffset);

  encoder_left_.init(ENCODER_LEFT_A_PIN, true);
  encoder_right_.init(ENCODER_RIGHT_A_PIN, false);
  last_micros_ = micros();
  MsTimer2::set(kBalanceIntervalMs, balanceISR);
  MsTimer2::start();
}

void BalanceCar::balanceISR() {
  if (instance_)
    instance_->balance();
}

void BalanceCar::balance() {
  // PORT(RPi): sei() is AVR-specific (re-enable global interrupts).
  // On RPi this ISR becomes a timer thread; remove sei() and use
  // std::mutex or RT scheduling instead of the re-entrance guard.
  sei();

  // Measure actual elapsed time
  uint32_t now = micros();
  float dt = (now - last_micros_) * 1e-6f;
  last_micros_ = now;
  if (dt <= 0 || dt > kDtMax)
    dt = kDtDefault;  // clamp on first call / overflow

  encoder_left_.update(pwm_left_);
  encoder_right_.update(pwm_right_);

  int16_t ax, ay, az, gx, gy, gz;
  mpu_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.process(ax, ay, az, gx, gy, gz, dt);
  kalmanfilter_angle = kalmanfilter.angle;

  // Balance PID (Ki=0, dt has no effect on integral)
  float balance_control_output =
      pid_balance.compute(kalmanfilter_angle - angle_zero_, 0,
                          kalmanfilter.gyro_x - angular_velocity_zero_);

  // Speed PID (every 8 cycles, ~40ms)
  speed_dt_ += dt;
  speed_control_period_count_++;
  if (speed_control_period_count_ >= kSpeedPidPeriod) {
    speed_control_period_count_ = 0;
    float car_speed =
        (encoder_left_.readAndReset() + encoder_right_.readAndReset()) * 0.5;
    speed_filter_ = speed_filter_ * (1.0f - kSpeedFilterAlpha) +
                    car_speed * kSpeedFilterAlpha;
    float speed_error = setting_car_speed - speed_filter_;
    speed_control_output_ =
        pid_speed.compute(-speed_filter_, speed_error, 0, speed_dt_);
    speed_dt_ = 0;
  }

  // Turn control (Ki=0, dt has no effect on integral)
  float rotation_control_output =
      pid_turn.compute(setting_turn_speed, 0, kalmanfilter.gyro_z);

  // Compute PWM
  pwm_left_ =
      balance_control_output - speed_control_output_ - rotation_control_output;
  pwm_right_ =
      balance_control_output - speed_control_output_ + rotation_control_output;
  pwm_left_ = constrain(pwm_left_, -kPwmLimit, kPwmLimit);
  pwm_right_ = constrain(pwm_right_, -kPwmLimit, kPwmLimit);

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
