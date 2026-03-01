#include "MotorDriver.h"

#include "Pins.h"

void MotorDriver::init() {
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  stop();
  enable();
}

void MotorDriver::stop() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void MotorDriver::enable() {
  digitalWrite(STBY_PIN, HIGH);
}

void MotorDriver::disable() {
  digitalWrite(STBY_PIN, LOW);
}

void MotorDriver::forward(uint8_t speed) {
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void MotorDriver::back(uint8_t speed) {
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void MotorDriver::drive(int16_t left, int16_t right) {
  if (left < 0) {
    digitalWrite(AIN1, 1);
    analogWrite(PWMA_LEFT, -left);
  } else {
    digitalWrite(AIN1, 0);
    analogWrite(PWMA_LEFT, left);
  }
  if (right < 0) {
    digitalWrite(BIN1, 1);
    analogWrite(PWMB_RIGHT, -right);
  } else {
    digitalWrite(BIN1, 0);
    analogWrite(PWMB_RIGHT, right);
  }
}
