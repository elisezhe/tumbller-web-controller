#include "VoltageMon.h"

#include "Pins.h"
#include "StatusLeds.h"

VoltageMon::VoltageMon(CarState& state)
    : state_(state),
      leds_(nullptr),
      low_voltage_flag_(1),
      vol_measure_time_(0) {
}

void VoltageMon::init(StatusLeds& leds) {
  leds_ = &leds;
  analogReference(INTERNAL);
}

void VoltageMon::measure() {
  if (millis() - vol_measure_time_ > 1000) {
    vol_measure_time_ = millis();
    double voltage =
        (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
    if (voltage > 7.8) {
      if (low_voltage_flag_ == 1) {
        leds_->lightOff();
        digitalWrite(STBY_PIN, HIGH);
      }
      low_voltage_flag_ = 0;
    } else {
      if (voltage < 7.0) {
        state_.motion_mode = STOP;
        digitalWrite(STBY_PIN, LOW);
      }
      if (state_.motion_mode == STANDBY || state_.motion_mode == STOP ||
          state_.motion_mode == START) {
        leds_->flash(StatusLeds::Color(255, 0, 0), DIR_ALL);
      }
      low_voltage_flag_ = 1;
    }
  }
}
