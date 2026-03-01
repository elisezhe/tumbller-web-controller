#include "Rgb.h"

#include "BalanceCar.h"
#include "Command.h"

void RGB::init(CarState& state, InputHandler& input, BalanceCar& car) {
  state_ = &state;
  input_ = &input;
  car_ = &car;
  begin();
  setBrightness(brightness);
  show();
}

bool RGB::rgbDelay(unsigned long wait) {
  rgb_delay_time = millis();
  while (millis() - rgb_delay_time < wait) {
    if (input_->getBluetoothData() || input_->getKeyValue()) {
      return true;
    }
    if (state_->motion_mode == STOP &&
        car_->balance_angle_min <= car_->kalmanfilter_angle &&
        car_->kalmanfilter_angle <= car_->balance_angle_max) {
      state_->motion_mode = STANDBY;
      lightOff();
      state_->function_mode = IDLE;
      return true;
    }
  }
  return false;
}
