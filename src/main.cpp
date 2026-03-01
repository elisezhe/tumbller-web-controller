#include <Arduino.h>

#include "BalanceCar.h"
#include "Command.h"
#include "Pins.h"
#include "Rgb.h"
#include "Ultrasonic.h"
#include "voltage.h"

class Tumbller {
 public:
  CarState state;
  InputHandler input;
  BalanceCar car;
  RGB rgb;
  UltrasonicSensor ultrasonic;
  VoltageMon voltage;

  unsigned long start_prev_time;
  unsigned long print_time;
  unsigned long start_time;

  Tumbller()
      : input(state),
        car(state),
        ultrasonic(state),
        voltage(state),
        start_prev_time(0),
        print_time(0),
        start_time(0) {
  }

  void setup() {
    Serial.begin(9600);
    ultrasonic.init(rgb);
    input.init();
    rgb.init(state, input, car);
    voltage.init(rgb);
    start_prev_time = millis();
    car.init(input);
  }

  void loop() {
    input.getKeyValue();
    input.getBluetoothData();
    keyEventHandle();
    ultrasonic.getDistance();
    voltage.measure();
    setMotionState();
    functionMode();
    ultrasonic.checkObstacle();
    rgb.blink(100);

    if (millis() - print_time > 100) {
      print_time = millis();
      Serial.println(car.kalmanfilter.angle);
    }

    if (millis() - start_time < 10) {
      state.function_mode = IDLE;
      state.motion_mode = STOP;
      car.stop();
    }

    if (millis() - start_time == 2000) {
      input.key_value = '5';
    }
  }

 private:
  void functionMode() {
    switch (state.function_mode) {
      case IDLE:
        break;
      case IRREMOTE:
        break;
      case OBSTACLE:
        ultrasonic.obstacleAvoidanceMode();
        break;
      case FOLLOW:
        ultrasonic.followMode();
        break;
      case BLUETOOTH:
        break;
      case FOLLOW2:
        ultrasonic.followMode2();
        break;
      default:
        break;
    }
  }

  void setMotionState() {
    switch (state.motion_mode) {
      case FORWARD:
        switch (state.function_mode) {
          case FOLLOW:
            car.setting_car_speed = 20;
            car.setting_turn_speed = 0;
            break;
          case FOLLOW2:
            car.setting_car_speed = 20;
            car.setting_turn_speed = 0;
            break;
          case BLUETOOTH:
            car.setting_car_speed = 80;
            break;
          case IRREMOTE:
            car.setting_car_speed = 80;
            car.setting_turn_speed = 0;
            break;
          default:
            car.setting_car_speed = 40;
            car.setting_turn_speed = 0;
            break;
        }
        break;
      case BACKWARD:
        switch (state.function_mode) {
          case FOLLOW:
            car.setting_car_speed = -20;
            car.setting_turn_speed = 0;
            break;
          case FOLLOW2:
            car.setting_car_speed = -20;
            car.setting_turn_speed = 0;
            break;
          case BLUETOOTH:
            car.setting_car_speed = -80;
            break;
          case IRREMOTE:
            car.setting_car_speed = -80;
            car.setting_turn_speed = 0;
            break;
          default:
            car.setting_car_speed = -40;
            car.setting_turn_speed = 0;
            break;
        }
        break;
      case TURNLEFT:
        switch (state.function_mode) {
          case FOLLOW:
            car.setting_car_speed = 0;
            car.setting_turn_speed = 50;
            break;
          case FOLLOW2:
            car.setting_car_speed = 0;
            car.setting_turn_speed = 50;
            break;
          case BLUETOOTH:
            car.setting_turn_speed = 80;
            break;
          case IRREMOTE:
            car.setting_car_speed = 0;
            car.setting_turn_speed = 80;
            break;
          default:
            car.setting_car_speed = 0;
            car.setting_turn_speed = 50;
            break;
        }
        break;
      case TURNRIGHT:
        switch (state.function_mode) {
          case FOLLOW:
            car.setting_car_speed = 0;
            car.setting_turn_speed = -50;
            break;
          case FOLLOW2:
            car.setting_car_speed = 0;
            car.setting_turn_speed = -50;
            break;
          case BLUETOOTH:
            car.setting_turn_speed = -80;
            break;
          case IRREMOTE:
            car.setting_car_speed = 0;
            car.setting_turn_speed = -80;
            break;
          default:
            car.setting_car_speed = 0;
            car.setting_turn_speed = -50;
            break;
        }
        break;
      case STANDBY:
        car.setting_car_speed = 0;
        car.setting_turn_speed = 0;
        break;
      case STOP:
        if (millis() - start_prev_time > 1000) {
          state.function_mode = IDLE;
          if (car.balance_angle_min <= car.kalmanfilter_angle &&
              car.kalmanfilter_angle <= car.balance_angle_max) {
            state.motion_mode = STANDBY;
            rgb.lightOff();
          }
        }
        break;
      case START:
        if (millis() - start_prev_time > 2000) {
          if (car.balance_angle_min <= car.kalmanfilter_angle &&
              car.kalmanfilter_angle <= car.balance_angle_max) {
            car.car_speed_integeral = 0;
            car.setting_car_speed = 0;
            state.motion_mode = STANDBY;
            rgb.lightOff();
          } else {
            state.motion_mode = STOP;
            car.stop();
            rgb.brightRedColor();
          }
        }
        break;
      default:
        break;
    }
  }

  void keyEventHandle() {
    if (input.key_value != '\0') {
      input.key_flag = input.key_value;
      switch (input.key_value) {
        case 's':
          rgb.lightOff();
          state.motion_mode = STANDBY;
          break;
        case 'f':
          rgb.flashBlueColorFront();
          state.motion_mode = FORWARD;
          break;
        case 'b':
          rgb.flashBlueColorback();
          state.motion_mode = BACKWARD;
          break;
        case 'l':
          rgb.flashBlueColorLeft();
          state.motion_mode = TURNLEFT;
          break;
        case 'i':
          rgb.flashBlueColorRight();
          state.motion_mode = TURNRIGHT;
          break;
        case '1':
          state.function_mode = FOLLOW;
          ultrasonic.follow_flag = 0;
          ultrasonic.follow_prev_time = millis();
          break;
        case '2':
          state.function_mode = OBSTACLE;
          ultrasonic.obstacle_avoidance_flag = 0;
          ultrasonic.obstacle_avoidance_prev_time = millis();
          break;
        case '3':
        rgb_loop:
          input.key_value = '\0';
          rgb.flag++;
          if (rgb.flag > 6) {
            rgb.flag = 1;
          }
          switch (rgb.flag) {
            case 0:
              break;
            case 1:
              if (rgb.theaterChaseRainbow(50) && input.key_value == '3')
                goto rgb_loop;
              break;
            case 2:
              if (rgb.rainbowCycle(20) && input.key_value == '3')
                goto rgb_loop;
              break;
            case 3:
              if (rgb.theaterChase(127, 127, 127, 50) && input.key_value == '3')
                goto rgb_loop;
              break;
            case 4:
              if (rgb.rainbow(20) && input.key_value == '3')
                goto rgb_loop;
              break;
            case 5:
              if (rgb.whiteOverRainbow(20, 30, 4) && input.key_value == '3')
                goto rgb_loop;
              break;
            case 6:
              if (rgb.rainbowFade2White(3, 50, 50) && input.key_value == '3')
                goto rgb_loop;
              break;
              break;
            default:
              break;
          }
          break;
        case '4':
          state.function_mode = IDLE;
          state.motion_mode = STOP;
          car.back(110);
          delay((car.kalmanfilter_angle - 30) * (car.kalmanfilter_angle - 30) /
                8);
          car.stop();
          start_prev_time = millis();
          rgb.brightRedColor();
          break;
        case '5':
          if (millis() - start_prev_time > 500 &&
              car.kalmanfilter_angle >= car.balance_angle_min) {
            start_prev_time = millis();
            state.motion_mode = START;
          }
          state.motion_mode = START;
          break;
        case '6':
          rgb.brightness = 50;
          rgb.setBrightness(rgb.brightness);
          rgb.show();
          break;
        case '7':
          rgb.brightRedColor();
          rgb.brightness -= 25;
          if (rgb.brightness <= 0) {
            rgb.brightness = 0;
          }
          rgb.setBrightness(rgb.brightness);
          rgb.show();
          break;
        case '8':
          rgb.brightRedColor();
          rgb.brightness += 25;
          if (rgb.brightness >= 255) {
            rgb.brightness = 255;
          }
          rgb.setBrightness(rgb.brightness);
          rgb.show();
          break;
        case '9':
          rgb.brightness = 0;
          rgb.setBrightness(rgb.brightness);
          rgb.show();
          break;
        case '0':
          state.function_mode = FOLLOW2;
          ultrasonic.follow_flag = 0;
          ultrasonic.follow_prev_time = millis();
          break;
        case '*':
          break;
        case '#':
          break;
        default:
          break;
      }
      if (input.key_flag == input.key_value) {
        input.key_value = '\0';
      }
    }
  }
};

Tumbller app;

void setup() {
  app.setup();
}
void loop() {
  app.loop();
}
