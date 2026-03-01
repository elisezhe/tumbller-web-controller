#include "BalanceCar.h"

#include "EnableInterruptWrapper.h"
#include "I2Cdev.h"
#include "InputHandler.h"
#include "MsTimer2.h"
#include "Pins.h"
#include "Wire.h"

BalanceCar* BalanceCar::instance_ = nullptr;

BalanceCar::BalanceCar(CarState& state)
    : kalmanfilter(),
      kalmanfilter_angle(0),
      balance_angle_min(-22),
      balance_angle_max(22),
      car_speed_integeral(0),
      setting_car_speed(0),
      setting_turn_speed(0),
      state_(state),
      input_(nullptr),
      mpu_(),
      kp_balance_(55),
      kd_balance_(0.75),
      kp_speed_(10),
      ki_speed_(0.26),
      kp_turn_(2.5),
      kd_turn_(0.5),
      angle_zero_(0),
      angular_velocity_zero_(0),
      encoder_count_right_a_(0),
      encoder_count_left_a_(0),
      ax_(0),
      ay_(0),
      az_(0),
      gx_(0),
      gy_(0),
      gz_(0),
      dt_(0.005),
      Q_angle_(0.001),
      Q_gyro_(0.005),
      R_angle_(0.5),
      C_0_(1),
      K1_(0.05),
      encoder_left_pulse_num_speed_(0),
      encoder_right_pulse_num_speed_(0),
      speed_control_output_(0),
      rotation_control_output_(0),
      speed_filter_(0),
      speed_control_period_count_(0),
      speed_filter_old_(0),
      pwm_left_(0),
      pwm_right_(0) {
}

void BalanceCar::init(InputHandler& input) {
  instance_ = this;
  input_ = &input;

  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  stop();

  Wire.begin();
  mpu_.initialize();

  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderLeftISR,
                  CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderRightISR, CHANGE);
  MsTimer2::set(5, balanceISR);
  MsTimer2::start();
}

void BalanceCar::balanceISR() {
  if (instance_)
    instance_->balance();
}

void BalanceCar::encoderLeftISR() {
  if (instance_)
    instance_->encoder_count_left_a_++;
}

void BalanceCar::encoderRightISR() {
  if (instance_)
    instance_->encoder_count_right_a_++;
}

void BalanceCar::stop() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void BalanceCar::forward(unsigned char speed) {
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void BalanceCar::back(unsigned char speed) {
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void BalanceCar::balance() {
  sei();

  encoder_left_pulse_num_speed_ +=
      pwm_left_ < 0 ? -encoder_count_left_a_ : encoder_count_left_a_;
  encoder_right_pulse_num_speed_ +=
      pwm_right_ < 0 ? -encoder_count_right_a_ : encoder_count_right_a_;
  encoder_count_left_a_ = 0;
  encoder_count_right_a_ = 0;

  mpu_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
  kalmanfilter.Angle(ax_, ay_, az_, gx_, gy_, gz_, dt_, Q_angle_, Q_gyro_,
                     R_angle_, C_0_, K1_);
  kalmanfilter_angle = kalmanfilter.angle;

  // Balance PID
  double balance_control_output =
      kp_balance_ * (kalmanfilter_angle - angle_zero_) +
      kd_balance_ * (kalmanfilter.Gyro_x - angular_velocity_zero_);

  // Speed PID (every 8 cycles = 40ms)
  speed_control_period_count_++;
  if (speed_control_period_count_ >= 8) {
    speed_control_period_count_ = 0;
    double car_speed =
        (encoder_left_pulse_num_speed_ + encoder_right_pulse_num_speed_) * 0.5;
    encoder_left_pulse_num_speed_ = 0;
    encoder_right_pulse_num_speed_ = 0;
    speed_filter_ = speed_filter_old_ * 0.7 + car_speed * 0.3;
    speed_filter_old_ = speed_filter_;
    car_speed_integeral += speed_filter_;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output_ =
        -kp_speed_ * speed_filter_ - ki_speed_ * car_speed_integeral;
  }

  // Turn control
  rotation_control_output_ =
      setting_turn_speed + kd_turn_ * kalmanfilter.Gyro_z;

  // Compute PWM
  pwm_left_ =
      balance_control_output - speed_control_output_ - rotation_control_output_;
  pwm_right_ =
      balance_control_output - speed_control_output_ + rotation_control_output_;
  pwm_left_ = constrain(pwm_left_, -255, 255);
  pwm_right_ = constrain(pwm_right_, -255, 255);

  // Fall detection
  if (state_.motion_mode != START && state_.motion_mode != STOP &&
      (kalmanfilter_angle < balance_angle_min ||
       balance_angle_max < kalmanfilter_angle)) {
    state_.motion_mode = STOP;
    stop();
  }

  // Motor output
  if (state_.motion_mode == STOP && input_->key_flag != '4') {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left_ = 0;
    pwm_right_ = 0;
    stop();
  } else if (state_.motion_mode == STOP) {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left_ = 0;
    pwm_right_ = 0;
  } else {
    if (pwm_left_ < 0) {
      digitalWrite(AIN1, 1);
      analogWrite(PWMA_LEFT, -pwm_left_);
    } else {
      digitalWrite(AIN1, 0);
      analogWrite(PWMA_LEFT, pwm_left_);
    }
    if (pwm_right_ < 0) {
      digitalWrite(BIN1, 1);
      analogWrite(PWMB_RIGHT, -pwm_right_);
    } else {
      digitalWrite(BIN1, 0);
      analogWrite(PWMB_RIGHT, pwm_right_);
    }
  }
}
