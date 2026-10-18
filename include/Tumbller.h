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

  uint32_t start_prev_time;
  uint32_t print_time;
  uint32_t start_time;

  Tumbller();
  void setup();
  void loop();

 private:
  void functionMode();
  void setMotionState();
  void keyEventHandle();
  void handleVoltage();
  void obstacleAvoidance();
  void followBehavior(uint8_t backup_dist, uint8_t follow_dist, uint8_t stop_dist);
  void startBehavior(FUNCTION_MODE mode);
  void cycleAnimation();
  void standUp();
  void standUpUpdate();
  void adjustBrightness(int16_t delta);
  void adjustDriveSpeed(int16_t delta);

  bool behavior_ready_;
  uint32_t behavior_start_time_;
  bool low_voltage_;
  bool auto_started_;
  bool stand_up_active_;
  uint32_t stand_up_start_;
  uint32_t stand_up_duration_;
  int16_t bluetooth_drive_speed_;
};

#endif
