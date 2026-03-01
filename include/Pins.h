#ifndef Pins_h
#define Pins_h

#define VOL_MEASURE_PIN A2
#define ECHO_PIN A3
#define TRIG_PIN 11
// RECV_PIN was pin 9 — removed: conflicts with IR_SEND_PIN (also pin 9)
// and is not referenced anywhere in the codebase.
#define NUMPIXELS 4
#define RGB_PIN 3
#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 12
#define PWMB_RIGHT 6
#define STBY_PIN 8
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_RIGHT_A_PIN 4
#define IR_SEND_PIN 9
#define LEFT_RECEIVE_PIN A0
#define RIGHT_RECEIVE_PIN A1
#define KEY_MODE 10
#endif
