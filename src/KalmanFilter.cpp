#include "KalmanFilter.h"

namespace {
const float kRadToDeg = 57.2957795f;  // 180/pi — radians to degrees
const float kGyroLsbPerDps = 131.0f;  // MPU6050 sensitivity at +/-250 deg/s range
}  // namespace

KalmanFilter::KalmanFilter()
    : angle(0), gyro_x(0), gyro_z(0), dt_(0), Q_angle_(0), Q_gyro_(0), R_angle_(0), C_0_(0), gyro_offset_(0), q_bias_(0) {
  memset(Pdot_, 0, sizeof(Pdot_));
  P_[0][0] = 1;
  P_[0][1] = 0;
  P_[1][0] = 0;
  P_[1][1] = 1;
}

void KalmanFilter::init(float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float gyro_offset) {
  dt_ = dt;
  Q_angle_ = Q_angle;
  Q_gyro_ = Q_gyro;
  R_angle_ = R_angle;
  C_0_ = C_0;
  gyro_offset_ = gyro_offset;
}

void KalmanFilter::process(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  float accel_angle = atan2(ay, az) * kRadToDeg;
  gyro_x = (gx - gyro_offset_) / kGyroLsbPerDps;
  update(accel_angle, gyro_x);
  gyro_z = -gz / kGyroLsbPerDps;
}

void KalmanFilter::process(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt) {
  dt_ = dt;
  process(ax, ay, az, gx, gy, gz);
}

void KalmanFilter::update(float angle_m, float gyro_m) {
  angle += (gyro_m - q_bias_) * dt_;
  float angle_err = angle_m - angle;
  Pdot_[0] = Q_angle_ - P_[0][1] - P_[1][0];
  Pdot_[1] = -P_[1][1];
  Pdot_[2] = -P_[1][1];
  Pdot_[3] = Q_gyro_;
  P_[0][0] += Pdot_[0] * dt_;
  P_[0][1] += Pdot_[1] * dt_;
  P_[1][0] += Pdot_[2] * dt_;
  P_[1][1] += Pdot_[3] * dt_;
  float PCt_0 = C_0_ * P_[0][0];
  float PCt_1 = C_0_ * P_[1][0];
  float E = R_angle_ + C_0_ * PCt_0;
  float K_0 = PCt_0 / E;
  float K_1 = PCt_1 / E;
  float t_0 = PCt_0;
  float t_1 = C_0_ * P_[0][1];
  P_[0][0] -= K_0 * t_0;
  P_[0][1] -= K_0 * t_1;
  P_[1][0] -= K_1 * t_0;
  P_[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  q_bias_ += K_1 * angle_err;
}
