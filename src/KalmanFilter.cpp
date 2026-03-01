#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
    : angle(0),
      gyro_x(0),
      gyro_z(0),
      dt_(0.005),
      Q_angle_(0.001),
      Q_gyro_(0.005),
      R_angle_(0.5),
      C_0_(1),
      gyro_offset_(128.1),
      angle_err_(0),
      q_bias_(0),
      PCt_0_(0),
      PCt_1_(0),
      E_(0),
      K_0_(0),
      K_1_(0),
      t_0_(0),
      t_1_(0),
      angle_dot_(0) {
  memset(Pdot_, 0, sizeof(Pdot_));
  P_[0][0] = 1;
  P_[0][1] = 0;
  P_[1][0] = 0;
  P_[1][1] = 1;
}

void KalmanFilter::init(float dt, float Q_angle, float Q_gyro, float R_angle,
                        float C_0, float gyro_offset) {
  dt_ = dt;
  Q_angle_ = Q_angle;
  Q_gyro_ = Q_gyro;
  R_angle_ = R_angle;
  C_0_ = C_0;
  gyro_offset_ = gyro_offset;
}

void KalmanFilter::process(int16_t ax, int16_t ay, int16_t az, int16_t gx,
                           int16_t gy, int16_t gz) {
  float accel_angle = atan2(ay, az) * 57.3;
  gyro_x = (gx - gyro_offset_) / 131.0;
  update(accel_angle, gyro_x);
  gyro_z = -gz / 131.0;
}

void KalmanFilter::update(double angle_m, double gyro_m) {
  angle += (gyro_m - q_bias_) * dt_;
  angle_err_ = angle_m - angle;
  Pdot_[0] = Q_angle_ - P_[0][1] - P_[1][0];
  Pdot_[1] = -P_[1][1];
  Pdot_[2] = -P_[1][1];
  Pdot_[3] = Q_gyro_;
  P_[0][0] += Pdot_[0] * dt_;
  P_[0][1] += Pdot_[1] * dt_;
  P_[1][0] += Pdot_[2] * dt_;
  P_[1][1] += Pdot_[3] * dt_;
  PCt_0_ = C_0_ * P_[0][0];
  PCt_1_ = C_0_ * P_[1][0];
  E_ = R_angle_ + C_0_ * PCt_0_;
  K_0_ = PCt_0_ / E_;
  K_1_ = PCt_1_ / E_;
  t_0_ = PCt_0_;
  t_1_ = C_0_ * P_[0][1];
  P_[0][0] -= K_0_ * t_0_;
  P_[0][1] -= K_0_ * t_1_;
  P_[1][0] -= K_1_ * t_0_;
  P_[1][1] -= K_1_ * t_1_;
  angle += K_0_ * angle_err_;
  q_bias_ += K_1_ * angle_err_;
  angle_dot_ = gyro_m - q_bias_;
}
