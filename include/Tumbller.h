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
};

#endif
