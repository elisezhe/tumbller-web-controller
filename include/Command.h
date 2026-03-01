#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>

#include "mode.h"

class InputHandler {
 public:
  InputHandler(CarState& state);
  void init();
  bool getKeyValue();
  bool getBluetoothData();

  char key_value;
  char key_flag;

 private:
  CarState& state_;
  char key_mode_;
  char prev_key_mode_;
  unsigned long key_mode_time_;

  void onKeyPress();

  static InputHandler* instance_;
  static void keyISR();
};

#endif
