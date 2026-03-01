#include "Tumbller.h"

#include "Pins.h"

namespace {
const unsigned long kSerialBaud = 9600;
const unsigned long kBlinkIntervalMs = 100;  // LED blink toggle period
const unsigned long kPrintIntervalMs = 100;  // serial debug output interval
const unsigned long kStartupGuardMs = 10;    // ignore input during first N ms
const unsigned long kAutoStartMs = 2000;     // auto-send '5' command after boot
const unsigned long kBehaviorWarmupMs =
    2000;  // settling time before autonomous mode
const unsigned long kStopRecoveryMs = 1000;  // STOP -> STANDBY recovery timeout
const unsigned long kStartSettleMs = 2000;  // START -> STANDBY settling timeout
const unsigned long kStartDebounceMs =
    500;                              // debounce for manual start command
const int kDefaultBrightness = 50;    // LED brightness on reset
const int kBrightnessStep = 25;       // brightness increment/decrement
const uint8_t kSelfRightSpeed = 110;  // motor PWM for self-right maneuver
const int8_t kSelfRightAngleRef =
    30;                               // reference angle for self-right duration
const uint8_t kSelfRightDivisor = 8;     // duration scaling divisor
const unsigned long kSelfRightMaxMs = 500;  // cap self-right motor duration
const uint8_t kObstacleNearCm = 20;      // back up if closer than this
const uint8_t kObstacleSlowCm = 40;   // stop forward if closer than this
}  // namespace

static Tumbller* tumbller_instance_ = nullptr;

static bool abortCheck() {
  Tumbller& t = *tumbller_instance_;
  if (t.input.getBluetoothData() || t.input.getKeyValue()) {
    return true;
  }
  if (t.state.motion_mode == STOP &&
      t.car.balance_angle_min <= t.car.kalmanfilter_angle &&
      t.car.kalmanfilter_angle <= t.car.balance_angle_max) {
    t.state.motion_mode = STANDBY;
    t.leds.lightOff();
    t.state.function_mode = IDLE;
    return true;
  }
  return false;
}

Tumbller::Tumbller()
    : input(state),
      car(state),
      start_prev_time(0),
      print_time(0),
      start_time(0),
      behavior_ready_(false),
      behavior_start_time_(0),
      low_voltage_(false),
      auto_started_(false) {
  tumbller_instance_ = this;
}

void Tumbller::setup() {
  Serial.begin(kSerialBaud);
  motors.init();
  ultrasonic.init();
  input.init();
  leds.init(abortCheck);
  voltage.init();
  start_prev_time = millis();
  start_time = millis();
  car.init(motors);
}

void Tumbller::loop() {
  input.getKeyValue();
  input.getBluetoothData();
  keyEventHandle();
  ultrasonic.getDistance();
  voltage.update();
  handleVoltage();
  setMotionState();
  functionMode();
  ultrasonic.checkObstacle();
  leds.blink(kBlinkIntervalMs);

  if (millis() - print_time > kPrintIntervalMs) {
    print_time = millis();
    Serial.println(car.kalmanfilter.angle);
  }

  if (millis() - start_time < kStartupGuardMs) {
    state.function_mode = IDLE;
    state.motion_mode = STOP;
    motors.stop();
  }

  if (!auto_started_ && millis() - start_time >= kAutoStartMs) {
    auto_started_ = true;
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
      if (millis() - behavior_start_time_ > kBehaviorWarmupMs) {
        behavior_ready_ = true;
      }
      if (behavior_ready_) {
        obstacleAvoidance();
      }
      break;
    case FOLLOW:
      if (millis() - behavior_start_time_ > kBehaviorWarmupMs) {
        behavior_ready_ = true;
      }
      if (behavior_ready_) {
        followBehavior(15, 15, 40);
      }
      break;
    case BLUETOOTH:
      break;
    case FOLLOW2:
      if (millis() - behavior_start_time_ > kBehaviorWarmupMs) {
        behavior_ready_ = true;
      }
      if (behavior_ready_) {
        followBehavior(10, 30, 50);
      }
      break;
    default:
      break;
  }
}

// Speed profile lookup table for directional motion modes.
// Indexed by [motion_mode - FORWARD][function_mode].
// KEEP sentinel means "do not change that axis" (used by Bluetooth).
namespace {
struct SpeedProfile {
  int8_t speed;
  int8_t turn;
};
static const int8_t KEEP = -128;
// Rows: FORWARD, BACKWARD, TURNLEFT, TURNRIGHT  (motion_mode 1-4)
// Cols: IDLE, IRREMOTE, OBSTACLE, FOLLOW, BLUETOOTH, FOLLOW2
static const PROGMEM SpeedProfile speed_profiles[4][6] = {
    // FORWARD
    {{40, 0}, {80, 0}, {40, 0}, {20, 0}, {80, KEEP}, {20, 0}},
    // BACKWARD
    {{-40, 0}, {-80, 0}, {-40, 0}, {-20, 0}, {-80, KEEP}, {-20, 0}},
    // TURNLEFT
    {{0, 50}, {0, 80}, {0, 50}, {0, 50}, {KEEP, 80}, {0, 50}},
    // TURNRIGHT
    {{0, -50}, {0, -80}, {0, -50}, {0, -50}, {KEEP, -80}, {0, -50}},
};
}  // namespace

void Tumbller::setMotionState() {
  switch (state.motion_mode) {
    case FORWARD:
    case BACKWARD:
    case TURNLEFT:
    case TURNRIGHT: {
      const uint8_t mi = static_cast<uint8_t>(state.motion_mode) - 1;
      const uint8_t fi = static_cast<uint8_t>(state.function_mode);
      SpeedProfile p;
      memcpy_P(&p, &speed_profiles[mi][fi], sizeof(p));
      if (p.speed != KEEP)
        car.setting_car_speed = p.speed;
      if (p.turn != KEEP)
        car.setting_turn_speed = p.turn;
      break;
    }
    case STANDBY:
      car.setting_car_speed = 0;
      car.setting_turn_speed = 0;
      break;
    case STOP:
      if (millis() - start_prev_time > kStopRecoveryMs) {
        state.function_mode = IDLE;
        if (car.balance_angle_min <= car.kalmanfilter_angle &&
            car.kalmanfilter_angle <= car.balance_angle_max) {
          state.motion_mode = STANDBY;
          leds.lightOff();
        }
      }
      break;
    case START:
      if (millis() - start_prev_time > kStartSettleMs) {
        if (car.balance_angle_min <= car.kalmanfilter_angle &&
            car.kalmanfilter_angle <= car.balance_angle_max) {
          car.pid_speed.resetIntegral();
          car.setting_car_speed = 0;
          state.motion_mode = STANDBY;
          leds.lightOff();
        } else {
          state.motion_mode = STOP;
          motors.stop();
          leds.solid(StatusLeds::Color(255, 0, 0));
        }
      }
      break;
    default:
      break;
  }
}

void Tumbller::keyEventHandle() {
  if (input.key_value == '\0')
    return;
  input.key_flag = input.key_value;

  switch (input.key_value) {
    // Motion commands
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

    // Mode selection
    case '1':
      startBehavior(FOLLOW);
      break;
    case '2':
      startBehavior(OBSTACLE);
      break;
    case '0':
      startBehavior(FOLLOW2);
      break;

    // LED animation cycle
    case '3':
      cycleAnimation();
      break;

    // Self-right maneuver
    case '4':
      selfRight();
      break;

    // Startup
    case '5':
      if (millis() - start_prev_time > kStartDebounceMs &&
          car.kalmanfilter_angle >= car.balance_angle_min) {
        start_prev_time = millis();
      }
      state.motion_mode = START;
      break;

    // Brightness control
    case '6':
      leds.brightness = kDefaultBrightness;
      leds.setBrightness(leds.brightness);
      leds.show();
      break;
    case '7':
      adjustBrightness(-kBrightnessStep);
      break;
    case '8':
      adjustBrightness(kBrightnessStep);
      break;
    case '9':
      leds.brightness = 0;
      leds.setBrightness(leds.brightness);
      leds.show();
      break;

    default:
      break;
  }

  if (input.key_flag == input.key_value) {
    input.key_value = '\0';
  }
}

void Tumbller::startBehavior(FUNCTION_MODE mode) {
  state.function_mode = mode;
  behavior_ready_ = false;
  behavior_start_time_ = millis();
}

void Tumbller::cycleAnimation() {
  do {
    input.key_value = '\0';
    leds.animation_index++;
    if (leds.animation_index > 6)
      leds.animation_index = 1;

    bool interrupted = false;
    switch (leds.animation_index) {
      case 1:
        interrupted = leds.theaterChaseRainbow(50);
        break;
      case 2:
        interrupted = leds.rainbowCycle(20);
        break;
      case 3:
        interrupted = leds.theaterChase(127, 127, 127, 50);
        break;
      case 4:
        interrupted = leds.rainbow(20);
        break;
      case 5:
        interrupted = leds.whiteOverRainbow(20, 30, 4);
        break;
      case 6:
        interrupted = leds.rainbowFade2White(3, 50, 50);
        break;
    }
    if (!interrupted || input.key_value != '3')
      break;
  } while (true);
}

void Tumbller::selfRight() {
  state.function_mode = IDLE;
  state.motion_mode = STOP;
  car.suppress_motor_output = true;
  motors.back(kSelfRightSpeed);
  float diff = car.kalmanfilter_angle - kSelfRightAngleRef;
  unsigned long dur = static_cast<unsigned long>(diff * diff / kSelfRightDivisor);
  if (dur > kSelfRightMaxMs)
    dur = kSelfRightMaxMs;
  delay(dur);
  motors.stop();
  car.suppress_motor_output = false;
  start_prev_time = millis();
  leds.solid(StatusLeds::Color(255, 0, 0));
}

void Tumbller::adjustBrightness(int delta) {
  leds.solid(StatusLeds::Color(255, 0, 0));
  leds.brightness = constrain(leds.brightness + delta, 0, 255);
  leds.setBrightness(leds.brightness);
  leds.show();
}

void Tumbller::obstacleAvoidance() {
  if (ultrasonic.left_is_obstacle && ultrasonic.right_is_obstacle) {
    state.motion_mode = BACKWARD;
    leds.flash(StatusLeds::Color(255, 255, 0), DIR_BACK);
  } else if (ultrasonic.left_is_obstacle) {
    state.motion_mode = TURNRIGHT;
    leds.flash(StatusLeds::Color(255, 255, 0), DIR_RIGHT);
  } else if (ultrasonic.right_is_obstacle) {
    state.motion_mode = TURNLEFT;
    leds.flash(StatusLeds::Color(255, 255, 0), DIR_LEFT);
  } else if (ultrasonic.distance_value < kObstacleNearCm) {
    state.motion_mode = BACKWARD;
    leds.flash(StatusLeds::Color(255, 255, 0), DIR_BACK);
  } else if (ultrasonic.distance_value < kObstacleSlowCm) {
    state.motion_mode = STANDBY;
    leds.lightOff();
  } else {
    state.motion_mode = FORWARD;
    leds.flash(StatusLeds::Color(255, 255, 0), DIR_FRONT);
  }
}

void Tumbller::followBehavior(uint8_t backup_dist, uint8_t follow_dist,
                              uint8_t stop_dist) {
  if (ultrasonic.left_is_obstacle && ultrasonic.right_is_obstacle) {
    state.motion_mode = FORWARD;
    leds.flash(StatusLeds::Color(0, 255, 0), DIR_FRONT);
  } else if (ultrasonic.left_is_obstacle) {
    state.motion_mode = TURNLEFT;
    leds.flash(StatusLeds::Color(0, 255, 0), DIR_LEFT);
  } else if (ultrasonic.right_is_obstacle) {
    state.motion_mode = TURNRIGHT;
    leds.flash(StatusLeds::Color(0, 255, 0), DIR_RIGHT);
  } else if (ultrasonic.distance_value < backup_dist) {
    state.motion_mode = BACKWARD;
    leds.flash(StatusLeds::Color(0, 255, 0), DIR_BACK);
  } else if (ultrasonic.distance_value < follow_dist) {
    state.motion_mode = FORWARD;
    leds.flash(StatusLeds::Color(0, 255, 0), DIR_FRONT);
  } else if (ultrasonic.distance_value < stop_dist) {
    state.motion_mode = STANDBY;
    leds.lightOff();
  } else {
    state.motion_mode = STANDBY;
    leds.lightOff();
  }
}

void Tumbller::handleVoltage() {
  VoltageLevel v = voltage.level();
  if (v == VOLTAGE_OK) {
    if (low_voltage_) {
      leds.lightOff();
      motors.enable();
      low_voltage_ = false;
    }
  } else {
    if (v == VOLTAGE_CRITICAL) {
      state.motion_mode = STOP;
      motors.disable();
    }
    if (state.motion_mode == STANDBY || state.motion_mode == STOP ||
        state.motion_mode == START) {
      leds.flash(StatusLeds::Color(255, 0, 0), DIR_ALL);
    }
    low_voltage_ = true;
  }
}
