#include "Tumbller.h"

#include "Pins.h"

Tumbller::Tumbller()
    : input(state),
      car(state),
      ultrasonic(state),
      voltage(state),
      start_prev_time(0),
      print_time(0),
      start_time(0) {
}

void Tumbller::setup() {
  Serial.begin(9600);
  ultrasonic.init(leds);
  input.init();
  leds.init(state, input, car);
  voltage.init(leds);
  start_prev_time = millis();
  car.init(input);
}

void Tumbller::loop() {
  input.getKeyValue();
  input.getBluetoothData();
  keyEventHandle();
  ultrasonic.getDistance();
  voltage.measure();
  setMotionState();
  functionMode();
  ultrasonic.checkObstacle();
  leds.blink(100);

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

void Tumbller::functionMode() {
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

void Tumbller::setMotionState() {
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
          leds.lightOff();
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
          leds.lightOff();
        } else {
          state.motion_mode = STOP;
          car.stop();
          leds.solid(StatusLeds::Color(255, 0, 0));
        }
      }
      break;
    default:
      break;
  }
}

void Tumbller::keyEventHandle() {
  if (input.key_value != '\0') {
    input.key_flag = input.key_value;
    switch (input.key_value) {
      case 's':
        leds.lightOff();
        state.motion_mode = STANDBY;
        break;
      case 'f':
        leds.flash(StatusLeds::Color(0, 0, 255), DIR_FRONT);
        state.motion_mode = FORWARD;
        break;
      case 'b':
        leds.flash(StatusLeds::Color(0, 0, 255), DIR_BACK);
        state.motion_mode = BACKWARD;
        break;
      case 'l':
        leds.flash(StatusLeds::Color(0, 0, 255), DIR_LEFT);
        state.motion_mode = TURNLEFT;
        break;
      case 'i':
        leds.flash(StatusLeds::Color(0, 0, 255), DIR_RIGHT);
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
        leds.animation_index++;
        if (leds.animation_index > 6) {
          leds.animation_index = 1;
        }
        switch (leds.animation_index) {
          case 0:
            break;
          case 1:
            if (leds.theaterChaseRainbow(50) && input.key_value == '3')
              goto rgb_loop;
            break;
          case 2:
            if (leds.rainbowCycle(20) && input.key_value == '3')
              goto rgb_loop;
            break;
          case 3:
            if (leds.theaterChase(127, 127, 127, 50) && input.key_value == '3')
              goto rgb_loop;
            break;
          case 4:
            if (leds.rainbow(20) && input.key_value == '3')
              goto rgb_loop;
            break;
          case 5:
            if (leds.whiteOverRainbow(20, 30, 4) && input.key_value == '3')
              goto rgb_loop;
            break;
          case 6:
            if (leds.rainbowFade2White(3, 50, 50) && input.key_value == '3')
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
        leds.solid(StatusLeds::Color(255, 0, 0));
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
        leds.brightness = 50;
        leds.setBrightness(leds.brightness);
        leds.show();
        break;
      case '7':
        leds.solid(StatusLeds::Color(255, 0, 0));
        leds.brightness -= 25;
        if (leds.brightness <= 0) {
          leds.brightness = 0;
        }
        leds.setBrightness(leds.brightness);
        leds.show();
        break;
      case '8':
        leds.solid(StatusLeds::Color(255, 0, 0));
        leds.brightness += 25;
        if (leds.brightness >= 255) {
          leds.brightness = 255;
        }
        leds.setBrightness(leds.brightness);
        leds.show();
        break;
      case '9':
        leds.brightness = 0;
        leds.setBrightness(leds.brightness);
        leds.show();
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
