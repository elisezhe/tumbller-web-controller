#include "InputHandler.h"

#include "EnableInterruptWrapper.h"
#include "Pins.h"

namespace {
const uint32_t kKeyDebounceMs = 500;  // minimum time between key presses
const uint8_t kNumKeyModes = 5;       // number of key-button cycling modes
}  // namespace

InputHandler* InputHandler::instance_ = nullptr;

InputHandler::InputHandler(CarState& state)
    : key_value('\0'), key_flag('\0'), state_(state), key_mode_(0), prev_key_mode_(0), key_mode_time_(0) {
}

void InputHandler::init() {
  instance_ = this;
  pinMode(KEY_MODE, INPUT_PULLUP);
  enableInterrupt(KEY_MODE, keyISR, FALLING);
}

void InputHandler::keyISR() {
  if (instance_)
    instance_->onKeyPress();
}

void InputHandler::onKeyPress() {
  if (millis() - key_mode_time_ > kKeyDebounceMs) {
    key_mode_time_ = millis();
    key_mode_++;
    if (key_mode_ >= kNumKeyModes) {
      key_mode_ = 0;
    }
  }
}

bool InputHandler::getKeyValue() {
  uint8_t mode = key_mode_;  // atomic snapshot (8-bit on AVR)
  if (prev_key_mode_ != mode) {
    prev_key_mode_ = mode;
    switch (mode) {
      case 0:
        key_value = 's';
        state_.function_mode = IDLE;
        break;
      case 1:
        key_value = '1';
        break;
      case 2:
        key_value = '2';
        break;
      case 3:
        key_value = '0';
        break;
      case 4:
        key_value = '3';
        break;
      default:
        break;
    }
    return true;
  }
  return false;
}

bool InputHandler::getBluetoothData() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\0' && c != '\n') {
      key_value = c;
      if (c == 'f' || c == 'b' || c == 'l' || c == 'i') {
        state_.function_mode = BLUETOOTH;
      }
      if (c == 's') {
        state_.function_mode = IDLE;
      }
      if (key_value == 'f' || key_value == 'b' || key_value == 'l' || key_value == 'i' || key_value == 's' || key_value == '0' ||
          key_value == '1' || key_value == '2' || key_value == '3' || key_value == '4' || key_value == '5' || key_value == '6' ||
          key_value == '7' || key_value == '8' || key_value == '9') {
        return true;
      }
    }
  }
  return false;
}
