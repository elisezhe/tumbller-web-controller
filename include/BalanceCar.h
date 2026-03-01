/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#ifndef BALANCECAR_H
#define BALANCECAR_H

#include <Arduino.h>

#include "KalmanFilter.h"
#include "MPU6050.h"

// PID parameters
extern double kp_balance, kd_balance;
extern double kp_speed, ki_speed;
extern double kp_turn, kd_turn;

// MPU6050 calibration
extern double angle_zero;
extern double angular_velocity_zero;

// Encoder counts
extern volatile unsigned long encoder_count_right_a;
extern volatile unsigned long encoder_count_left_a;

// Control variables
extern int encoder_left_pulse_num_speed;
extern int encoder_right_pulse_num_speed;
extern double speed_control_output;
extern double rotation_control_output;
extern double speed_filter;
extern int speed_control_period_count;
extern double car_speed_integeral;
extern double speed_filter_old;
extern int setting_car_speed;
extern int setting_turn_speed;
extern double pwm_left;
extern double pwm_right;
extern float kalmanfilter_angle;
extern char balance_angle_min;
extern char balance_angle_max;

extern MPU6050 mpu;
extern KalmanFilter kalmanfilter;

void carStop();
void carForward(unsigned char speed);
void carBack(unsigned char speed);
void balanceCar();
void encoderCountRightA();
void encoderCountLeftA();
void carInitialize();

#endif
