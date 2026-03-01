#include "UltrasonicSensor.h"

#include "EnableInterruptWrapper.h"
#include "Pins.h"
#include "StatusLeds.h"

UltrasonicSensor* UltrasonicSensor::instance_ = nullptr;

UltrasonicSensor::UltrasonicSensor(CarState& state)
    : obstacle_avoidance_flag(0),
      obstacle_avoidance_prev_time(0),
      follow_flag(0),
      follow_prev_time(0),
      distance_value(0),
      state_(state),
      leds_(nullptr),
      left_count_flag_(0),
      left_index_(0),
      right_count_flag_(0),
      right_index_(0),
      measure_flag_(0),
      measure_prev_time_(0),
      get_distance_prev_time_(0),
      turn_count_(0),
      ir_send_time_(0),
      left_receive_flag_(0),
      left_count_(0),
      left_count_time_(0),
      left_is_obstacle_(false),
      right_receive_flag_(0),
      right_count_(0),
      right_count_time_(0),
      right_is_obstacle_(false) {
  memset(left_flag_, 0, sizeof(left_flag_));
  memset(right_flag_, 0, sizeof(right_flag_));
}

void UltrasonicSensor::init(StatusLeds& leds) {
  instance_ = this;
  leds_ = &leds;
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
    distance_value = (micros() - measure_prev_time_) / 58.2;
    if (distance_value > 600)
      distance_value = 600;
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
  for (unsigned char i = 0; i < 10; i++) {
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
  for (unsigned char i = 0; i < 10; i++) {
    if (left_flag_[i])
      left_count_flag_++;
  }
}

void UltrasonicSensor::send38K(int pin) {
  for (int i = 0; i < 39; i++) {
    digitalWrite(pin, LOW);
    delayMicroseconds(9);
    digitalWrite(pin, HIGH);
    delayMicroseconds(9);
  }
}

void UltrasonicSensor::getDistance() {
  if (millis() - get_distance_prev_time_ > 100) {
    get_distance_prev_time_ = millis();
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    measure_flag_ = 0;
    attachPinChangeInterrupt(ECHO_PIN, measureDistanceISR, RISING);
  }
}

void UltrasonicSensor::checkObstacle() {
  if (millis() - ir_send_time_ > 50) {
    ir_send_time_ = millis();
    send38K(IR_SEND_PIN);
    delay(2);

    if (left_receive_flag_ == 0) {
      left_receive_flag_ = 1;
      left_count_ = 0;
      left_count_time_ = millis();
    }
    if (millis() - left_count_time_ > 50) {
      left_receive_flag_ = 0;
      if (left_count_ >= 2) {
        leftFilter(true);
      } else {
        leftFilter(false);
      }
    }
    left_is_obstacle_ = (left_count_flag_ >= 3);

    if (right_receive_flag_ == 0) {
      right_receive_flag_ = 1;
      right_count_ = 0;
      right_count_time_ = millis();
    }
    if (millis() - right_count_time_ > 50) {
      right_receive_flag_ = 0;
      if (right_count_ >= 2) {
        rightFilter(true);
      } else {
        rightFilter(false);
      }
    }
    right_is_obstacle_ = (right_count_flag_ >= 3);
  }
}

void UltrasonicSensor::obstacleAvoidanceMode() {
  if (millis() - obstacle_avoidance_prev_time > 2000) {
    obstacle_avoidance_flag = 1;
  }
  if (obstacle_avoidance_flag == 1) {
    if (left_is_obstacle_ && right_is_obstacle_) {
      state_.motion_mode = BACKWARD;
      leds_->flash(StatusLeds::Color(255, 255, 0), DIR_BACK);
    } else if (left_is_obstacle_) {
      state_.motion_mode = TURNRIGHT;
      leds_->flash(StatusLeds::Color(255, 255, 0), DIR_RIGHT);
    } else if (right_is_obstacle_) {
      state_.motion_mode = TURNLEFT;
      leds_->flash(StatusLeds::Color(255, 255, 0), DIR_LEFT);
    } else if (distance_value < 20) {
      state_.motion_mode = BACKWARD;
      leds_->flash(StatusLeds::Color(255, 255, 0), DIR_BACK);
    } else if (distance_value >= 20 && distance_value < 40) {
      state_.motion_mode = STANDBY;
      leds_->lightOff();
    } else {
      state_.motion_mode = FORWARD;
      leds_->flash(StatusLeds::Color(255, 255, 0), DIR_FRONT);
    }
  }
}

void UltrasonicSensor::followMode() {
  if (millis() - follow_prev_time > 2000) {
    follow_flag = 1;
  }
  if (follow_flag == 1) {
    if (left_is_obstacle_ && right_is_obstacle_) {
      state_.motion_mode = FORWARD;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_FRONT);
    } else if (left_is_obstacle_) {
      state_.motion_mode = TURNLEFT;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_LEFT);
    } else if (right_is_obstacle_) {
      state_.motion_mode = TURNRIGHT;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_RIGHT);
    } else if (distance_value < 15) {
      state_.motion_mode = BACKWARD;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_BACK);
    } else if (distance_value >= 15 && distance_value < 40) {
      state_.motion_mode = STANDBY;
      leds_->lightOff();
    } else {
      state_.motion_mode = STANDBY;
      leds_->lightOff();
    }
  }
}

void UltrasonicSensor::followMode2() {
  if (millis() - follow_prev_time > 2000) {
    follow_flag = 1;
  }
  if (follow_flag == 1) {
    if (left_is_obstacle_ && right_is_obstacle_) {
      state_.motion_mode = FORWARD;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_FRONT);
    } else if (left_is_obstacle_) {
      state_.motion_mode = TURNLEFT;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_LEFT);
    } else if (right_is_obstacle_) {
      state_.motion_mode = TURNRIGHT;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_RIGHT);
    } else if (distance_value < 10) {
      state_.motion_mode = BACKWARD;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_BACK);
    } else if (distance_value >= 10 && distance_value < 30) {
      state_.motion_mode = FORWARD;
      leds_->flash(StatusLeds::Color(0, 255, 0), DIR_FRONT);
    } else if (distance_value >= 30 && distance_value < 50) {
      state_.motion_mode = STANDBY;
      leds_->lightOff();
    } else {
      state_.motion_mode = STANDBY;
      leds_->lightOff();
    }
  }
}
