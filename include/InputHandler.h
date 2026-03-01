#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

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
  // PORT(RPi): replace with std::atomic<char> — volatile char is atomic
  // on AVR (single byte) but needs proper atomics on ARM.
  volatile char key_mode_;  // written from ISR, read from loop()
  char prev_key_mode_;
  unsigned long key_mode_time_;

  void onKeyPress();

  static InputHandler* instance_;
  static void keyISR();
};

#endif
