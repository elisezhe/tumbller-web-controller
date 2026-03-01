#include "UltrasonicSensor.h"

#include "EnableInterruptWrapper.h"
#include "Pins.h"

namespace {
const float kSoundUsPerCm = 58.2;          // round-trip microseconds per cm
const float kMaxDistanceCm = 600.0;        // cap readings beyond this
const uint32_t kDistanceIntervalMs = 100;  // minimum ms between distance pings
const uint32_t kObstacleCheckIntervalMs =
    50;                             // minimum ms between IR obstacle checks
const uint8_t kIrPulseCount = 39;   // pulses per 38 kHz burst (~1ms)
const uint8_t kIrHalfPeriodUs = 9;  // half-period for ~38 kHz carrier
const uint8_t kIrSettleMs = 2;      // settle time after IR burst
const uint8_t kObstacleHitThreshold = 2;  // min ISR hits to flag obstacle
const uint8_t kFilterMajority =
    3;  // min positive samples (out of 10) to declare
const uint32_t kObstacleWindowMs = 50;  // sliding window for hit counting
const uint8_t kTrigSetupUs = 2;   // HC-SR04: low before trigger pulse
const uint8_t kTrigPulseUs = 10;  // HC-SR04: trigger pulse width
}  // namespace

UltrasonicSensor* UltrasonicSensor::instance_ = nullptr;

UltrasonicSensor::UltrasonicSensor()
    : distance_value(0),
      left_is_obstacle(false),
      right_is_obstacle(false),
      left_count_flag_(0),
      left_index_(0),
      right_count_flag_(0),
      right_index_(0),
      measure_flag_(0),
      measure_prev_time_(0),
      get_distance_prev_time_(0),
      ir_send_time_(0),
      left_receive_flag_(0),
      left_count_(0),
      left_count_time_(0),
      right_receive_flag_(0),
      right_count_(0),
      right_count_time_(0) {
  memset(left_flag_, 0, sizeof(left_flag_));
  memset(right_flag_, 0, sizeof(right_flag_));
}

void UltrasonicSensor::init() {
  instance_ = this;
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  enableInterrupt(LEFT_RECEIVE_PIN, leftReceiveISR, FALLING);
  enableInterrupt(RIGHT_RECEIVE_PIN, rightReceiveISR, FALLING);
}

void UltrasonicSensor::measureDistanceISR() {
  if (instance_)
    instance_->measureDistanceHandler();
}

void UltrasonicSensor::leftReceiveISR() {
  if (instance_)
    instance_->leftReceiveHandler();
}

void UltrasonicSensor::rightReceiveISR() {
  if (instance_)
    instance_->rightReceiveHandler();
}

void UltrasonicSensor::measureDistanceHandler() {
  if (measure_flag_ == 0) {
    measure_prev_time_ = micros();
    attachPinChangeInterrupt(ECHO_PIN, measureDistanceISR, FALLING);
    measure_flag_ = 1;
  } else if (measure_flag_ == 1) {
    measure_flag_ = 2;
    distance_value = (micros() - measure_prev_time_) / kSoundUsPerCm;
    if (distance_value > kMaxDistanceCm)
      distance_value = kMaxDistanceCm;
  }
}

void UltrasonicSensor::leftReceiveHandler() {
  left_count_++;
}

void UltrasonicSensor::rightReceiveHandler() {
  right_count_++;
}

void UltrasonicSensor::rightFilter(bool value) {
  right_flag_[right_index_] = value;
  right_index_++;
  if (right_index_ >= 10)
    right_index_ = 0;
  right_count_flag_ = 0;
  for (uint8_t i = 0; i < 10; i++) {
    if (right_flag_[i])
      right_count_flag_++;
  }
}

void UltrasonicSensor::leftFilter(bool value) {
  left_flag_[left_index_] = value;
  left_index_++;
  if (left_index_ >= 10)
    left_index_ = 0;
  left_count_flag_ = 0;
  for (uint8_t i = 0; i < 10; i++) {
    if (left_flag_[i])
      left_count_flag_++;
  }
}

void UltrasonicSensor::send38K(uint8_t pin) {
  for (uint8_t i = 0; i < kIrPulseCount; i++) {
    digitalWrite(pin, LOW);
    delayMicroseconds(kIrHalfPeriodUs);
    digitalWrite(pin, HIGH);
    delayMicroseconds(kIrHalfPeriodUs);
  }
}

void UltrasonicSensor::getDistance() {
  if (millis() - get_distance_prev_time_ > kDistanceIntervalMs) {
    get_distance_prev_time_ = millis();
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(kTrigSetupUs);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(kTrigPulseUs);
    digitalWrite(TRIG_PIN, LOW);
    measure_flag_ = 0;
    attachPinChangeInterrupt(ECHO_PIN, measureDistanceISR, RISING);
  }
}

void UltrasonicSensor::checkObstacle() {
  if (millis() - ir_send_time_ > kObstacleCheckIntervalMs) {
    ir_send_time_ = millis();
    send38K(IR_SEND_PIN);
    delay(kIrSettleMs);  // blocking: 2ms settle for IR receiver

    if (left_receive_flag_ == 0) {
      left_receive_flag_ = 1;
      left_count_ = 0;
      left_count_time_ = millis();
    }
    if (millis() - left_count_time_ > kObstacleWindowMs) {
      left_receive_flag_ = 0;
      leftFilter(left_count_ >= kObstacleHitThreshold);
    }
    left_is_obstacle = (left_count_flag_ >= kFilterMajority);

    if (right_receive_flag_ == 0) {
      right_receive_flag_ = 1;
      right_count_ = 0;
      right_count_time_ = millis();
    }
    if (millis() - right_count_time_ > kObstacleWindowMs) {
      right_receive_flag_ = 0;
      rightFilter(right_count_ >= kObstacleHitThreshold);
    }
    right_is_obstacle = (right_count_flag_ >= kFilterMajority);
  }
}
