#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>

/// Pure proximity sensor — IR obstacle detection + ultrasonic distance.
/// No behavior logic, no state mutation, no LED control.
class UltrasonicSensor {
 public:
  UltrasonicSensor();
  void init();
  void checkObstacle();
  void getDistance();

  double distance_value;
  bool left_is_obstacle;
  bool right_is_obstacle;

 private:
  bool left_flag_[10];
  unsigned char left_count_flag_;
  unsigned char left_index_;
  bool right_flag_[10];
  unsigned char right_count_flag_;
  unsigned char right_index_;

  char measure_flag_;
  unsigned long measure_prev_time_;
  unsigned long get_distance_prev_time_;

  unsigned long ir_send_time_;
  unsigned char left_receive_flag_;
  unsigned int left_count_;
  unsigned long left_count_time_;

  unsigned char right_receive_flag_;
  unsigned int right_count_;
  unsigned long right_count_time_;

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
