/*
 * @Description: RGB implementation
 */
#include "Rgb.h"

#include "BalanceCar.h"
#include "Command.h"
#include "mode.h"

RGB rgb;

bool RGB::rgbDelay(unsigned long wait) {
  rgb_delay_time = millis();
  while (millis() - rgb_delay_time < wait) {
    if (getBluetoothData() || getKeyValue()) {
      return true;
    }
    if (motion_mode == STOP && balance_angle_min <= kalmanfilter_angle &&
        kalmanfilter_angle <= balance_angle_max) {
      motion_mode = STANDBY;
      lightOff();
      function_mode = IDLE;
      return true;
    }
  }
  return false;
}
