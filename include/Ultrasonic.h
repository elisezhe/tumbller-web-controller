/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-10 14:24:57
 * @LastEditors: Please set LastEditors
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

extern bool left_flag[10];
extern unsigned char left_count_flag;
extern unsigned char left_index;
extern bool right_flag[10];
extern unsigned char right_count_flag;
extern unsigned char right_index;

extern char measure_flag;
extern double distance_value;
extern unsigned long measure_prev_time;
extern unsigned long get_distance_prev_time;
extern char obstacle_avoidance_flag;
extern unsigned long obstacle_avoidance_prev_time;
extern int follow_flag;
extern unsigned long follow_prev_time;
extern long int turn_count;

extern unsigned long ir_send_time;
extern unsigned char left_receive_flag;
extern unsigned int left_count;
extern unsigned long left_count_time;
extern bool left_test_flag;
extern bool left_is_obstacle;

extern unsigned char right_receive_flag;
extern unsigned int right_count;
extern unsigned long right_count_time;
extern bool right_test_flag;
extern bool right_is_obstacle;

void rightFilter(bool value);
void leftFilter(bool value);
void send38K(int pin);
void leftReceive();
void rightReceive();
void ultrasonicInit();
void checkObstacle();
void obstacleAvoidanceMode();
void followMode();
void followMode2();
void measureDistance();
void getDistance();

#endif
