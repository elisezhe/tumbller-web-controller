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

  float distance_value;
  bool left_is_obstacle;
  bool right_is_obstacle;

 private:
  bool left_flag_[10];
  uint8_t left_count_flag_;
  uint8_t left_index_;
  bool right_flag_[10];
  uint8_t right_count_flag_;
  uint8_t right_index_;

  uint8_t measure_flag_;
  uint32_t measure_prev_time_;
  uint32_t get_distance_prev_time_;

  uint32_t ir_send_time_;
  uint8_t left_receive_flag_;
  // PORT(RPi): replace with std::atomic<uint16_t> — 16-bit
  // reads are non-atomic on AVR; on ARM use atomics or mutex.
  uint16_t left_count_;
  uint32_t left_count_time_;

  uint8_t right_receive_flag_;
  // PORT(RPi): same as left_count_ — use std::atomic<uint16_t>.
  uint16_t right_count_;
  uint32_t right_count_time_;

  void rightFilter(bool value);
  void leftFilter(bool value);
  void send38K(uint8_t pin);
  void measureDistanceHandler();
  void leftReceiveHandler();
  void rightReceiveHandler();

  static UltrasonicSensor* instance_;
  static void measureDistanceISR();
  static void leftReceiveISR();
  static void rightReceiveISR();
};

#endif
