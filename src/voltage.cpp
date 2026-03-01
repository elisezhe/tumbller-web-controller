/*
 * @Description: Voltage measurement implementation
 */
#include "voltage.h"

#include "Pins.h"
#include "Rgb.h"
#include "mode.h"

int low_voltage_flag = 1;
unsigned long vol_measure_time = 0;

void voltageInit() {
  analogReference(INTERNAL);
}

void voltageMeasure() {
  if (millis() - vol_measure_time > 1000) {
    vol_measure_time = millis();
    double voltage =
        (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
    if (voltage > 7.8) {
      if (low_voltage_flag == 1) {
        rgb.lightOff();
        digitalWrite(STBY_PIN, HIGH);
      }
      low_voltage_flag = 0;
    } else {
      if (voltage < 7.0) {
        motion_mode = STOP;
        digitalWrite(STBY_PIN, LOW);
      }
      if (motion_mode == STANDBY || motion_mode == STOP ||
          motion_mode == START) {
        rgb.flashRedColor();
      }
      low_voltage_flag = 1;
    }
  }
}
