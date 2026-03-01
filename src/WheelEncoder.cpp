#include "WheelEncoder.h"

#include "EnableInterruptWrapper.h"

WheelEncoder* WheelEncoder::instances_[2] = {nullptr, nullptr};
uint8_t WheelEncoder::instance_count_ = 0;

WheelEncoder::WheelEncoder() : ticks_(0), accumulated_(0) {
}

void WheelEncoder::init(uint8_t pin, bool use_pcint) {
  uint8_t idx = instance_count_++;
  instances_[idx] = this;

  void (*isr)() = (idx == 0) ? isr0 : isr1;
  uint8_t p = use_pcint ? (pin | PINCHANGEINTERRUPT) : pin;
  enableInterrupt(p, isr, CHANGE);
}

void WheelEncoder::update(double pwm) {
  unsigned long t = ticks_;
  ticks_ = 0;
  accumulated_ += (pwm < 0) ? -static_cast<int>(t) : static_cast<int>(t);
}

int WheelEncoder::readAndReset() {
  int val = accumulated_;
  accumulated_ = 0;
  return val;
}

void WheelEncoder::tick() {
  ticks_++;
}

void WheelEncoder::isr0() {
  if (instances_[0])
    instances_[0]->tick();
}

void WheelEncoder::isr1() {
  if (instances_[1])
    instances_[1]->tick();
}
