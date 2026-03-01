#include "VoltageMon.h"

#include "Pins.h"

namespace {
const unsigned long kMeasureIntervalMs = 1000;
const double kAdcRefVoltage = 1.1;     // ATmega328P INTERNAL reference (volts)
const double kAdcResolution = 1024.0;  // 10-bit ADC
const double kDividerR1 = 10.0;  // upper resistor in voltage divider (kOhm)
const double kDividerR2 = 1.5;   // lower resistor in voltage divider (kOhm)
const double kVoltageOk = 7.8;  // volts — above this = OK (2S Li-ion nominal)
const double kVoltageCritical =
    7.0;  // volts — below this = critical (2S cutoff)
}  // namespace

VoltageMon::VoltageMon() : level_(VOLTAGE_OK), measure_time_(0) {
}

void VoltageMon::init() {
  analogReference(INTERNAL);
}

void VoltageMon::update() {
  if (millis() - measure_time_ > kMeasureIntervalMs) {
    measure_time_ = millis();
    double voltage =
        (analogRead(VOL_MEASURE_PIN) * kAdcRefVoltage / kAdcResolution) *
        ((kDividerR1 + kDividerR2) / kDividerR2);
    if (voltage > kVoltageOk) {
      level_ = VOLTAGE_OK;
    } else if (voltage < kVoltageCritical) {
      level_ = VOLTAGE_CRITICAL;
    } else {
      level_ = VOLTAGE_LOW;
    }
  }
}
