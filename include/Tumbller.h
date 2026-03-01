#ifndef TUMBLLER_H
#define TUMBLLER_H

#include <Arduino.h>

#include "BalanceCar.h"
#include "InputHandler.h"
#include "MotorDriver.h"
#include "StatusLeds.h"
#include "UltrasonicSensor.h"
#include "VoltageMon.h"

class Tumbller {
 public:
  CarState state;
  InputHandler input;
  MotorDriver motors;
  BalanceCar car;
  StatusLeds leds;
  UltrasonicSensor ultrasonic;
  VoltageMon voltage;

  unsigned long start_prev_time;
  unsigned long print_time;
  unsigned long start_time;

  Tumbller();
  void setup();
  void loop();

 private:
  void functionMode();
  void setMotionState();
  void keyEventHandle();
  void handleVoltage();
  void obstacleAvoidance();
  void followBehavior(uint8_t backup_dist, uint8_t follow_dist,
                      uint8_t stop_dist);
  void startBehavior(FUNCTION_MODE mode);
  void cycleAnimation();
  void selfRight();
  void adjustBrightness(int delta);

  bool behavior_ready_;
  unsigned long behavior_start_time_;
  bool low_voltage_;
  bool auto_started_;
};

#endif
