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
  // PORT(RPi): replace with std::atomic<uint8_t> — volatile uint8_t is atomic
  // on AVR (single byte) but needs proper atomics on ARM.
  volatile uint8_t key_mode_;  // written from ISR, read from loop()
  uint8_t prev_key_mode_;
  uint32_t key_mode_time_;

  void onKeyPress();

  static InputHandler* instance_;
  static void keyISR();
};

#endif
