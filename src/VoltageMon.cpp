#include "VoltageMon.h"

#include "Pins.h"

VoltageMon::VoltageMon() : level_(VOLTAGE_OK), measure_time_(0) {
}

void VoltageMon::init() {
  analogReference(INTERNAL);
}

void VoltageMon::update() {
  if (millis() - measure_time_ > 1000) {
    measure_time_ = millis();
    double voltage =
        (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
    if (voltage > 7.8) {
      level_ = VOLTAGE_OK;
    } else if (voltage < 7.0) {
      level_ = VOLTAGE_CRITICAL;
    } else {
      level_ = VOLTAGE_LOW;
    }
  }
}
