#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

#include "mode.h"

class RGB;

class UltrasonicSensor {
 public:
  UltrasonicSensor(CarState& state);
  void init(RGB& rgb);
  void checkObstacle();
  void getDistance();
  void obstacleAvoidanceMode();
  void followMode();
  void followMode2();

  char obstacle_avoidance_flag;
  unsigned long obstacle_avoidance_prev_time;
  int follow_flag;
  unsigned long follow_prev_time;
  double distance_value;

 private:
  CarState& state_;
  RGB* rgb_;

  bool left_flag_[10];
  unsigned char left_count_flag_;
  unsigned char left_index_;
  bool right_flag_[10];
  unsigned char right_count_flag_;
  unsigned char right_index_;

  char measure_flag_;
  unsigned long measure_prev_time_;
  unsigned long get_distance_prev_time_;
  long int turn_count_;

  unsigned long ir_send_time_;
  unsigned char left_receive_flag_;
  unsigned int left_count_;
  unsigned long left_count_time_;
  bool left_is_obstacle_;

  unsigned char right_receive_flag_;
  unsigned int right_count_;
  unsigned long right_count_time_;
  bool right_is_obstacle_;

  void rightFilter(bool value);
  void leftFilter(bool value);
  void send38K(int pin);
  void measureDistanceHandler();
  void leftReceiveHandler();
  void rightReceiveHandler();

  static UltrasonicSensor* instance_;
  static void measureDistanceISR();
  static void leftReceiveISR();
  static void rightReceiveISR();
};

#endif
