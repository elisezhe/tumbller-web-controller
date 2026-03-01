#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Arduino.h>

class KalmanFilter {
 public:
  KalmanFilter();

  // Configure filter parameters (call once before use).
  void init(float dt, float Q_angle, float Q_gyro, float R_angle, float C_0,
            float gyro_offset);

  // Process raw IMU data -> updates angle, gyro_x, gyro_z.
  // Uses the dt configured via init().
  void process(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
               int16_t gz);

  // Process with explicit dt (seconds) for variable-rate loops.
  void process(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy,
               int16_t gz, float dt);

  float angle;   // Kalman-filtered tilt angle (degrees)
  float gyro_x;  // Processed X gyro rate (deg/s)
  float gyro_z;  // Processed Z gyro rate (deg/s)

 private:
  void update(float angle_m, float gyro_m);

  float dt_;
  float Q_angle_;
  float Q_gyro_;
  float R_angle_;
  float C_0_;
  float gyro_offset_;

  float q_bias_;
  float Pdot_[4];
  float P_[2][2];
};

#endif