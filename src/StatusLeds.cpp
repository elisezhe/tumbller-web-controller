#include "StatusLeds.h"

#include "BalanceCar.h"
#include "InputHandler.h"

StatusLeds::StatusLeds()
    : Adafruit_NeoPixel(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800),
      state_(nullptr),
      input_(nullptr),
      car_(nullptr),
      brightness(50),
      animation_index(0),
      blink_phase_(0),
      blink_prev_millis_(0),
      white_rainbow_last_time_(0) {
  memset(led_on_, 0, sizeof(led_on_));
  memset(led_off_, 0, sizeof(led_off_));
}

void StatusLeds::init(CarState& state, InputHandler& input, BalanceCar& car) {
  state_ = &state;
  input_ = &input;
  car_ = &car;
  begin();
  setBrightness(brightness);
  show();
}

// --- Indicator patterns ---

void StatusLeds::lightOff() {
  memset(led_on_, 0, sizeof(led_on_));
  memset(led_off_, 0, sizeof(led_off_));
}

void StatusLeds::solid(uint32_t color) {
  for (uint8_t i = 0; i < NUMPIXELS; i++) {
    led_on_[i] = color;
    led_off_[i] = color;
  }
}

void StatusLeds::flash(uint32_t color, LedDirection dir) {
  memset(led_on_, 0, sizeof(led_on_));
  memset(led_off_, 0, sizeof(led_off_));

  switch (dir) {
    case DIR_ALL:
      for (uint8_t i = 0; i < NUMPIXELS; i++)
        led_on_[i] = color;
      break;
    case DIR_FRONT:
      led_on_[2] = color;
      led_on_[3] = color;
      break;
    case DIR_BACK:
      led_on_[0] = color;
      led_on_[1] = color;
      break;
    case DIR_LEFT:
      led_on_[1] = color;
      led_on_[2] = color;
      break;
    case DIR_RIGHT:
      led_on_[0] = color;
      led_on_[3] = color;
      break;
  }
}

void StatusLeds::blink(unsigned long interval) {
  if ((millis() - blink_prev_millis_ < interval) && (blink_phase_ == 0)) {
    blink_phase_ = 1;
    for (uint8_t i = 0; i < NUMPIXELS; i++)
      setPixelColor(i, led_on_[i]);
    show();
  } else if ((millis() - blink_prev_millis_ < interval * 2) &&
             (millis() - blink_prev_millis_ > interval) &&
             (blink_phase_ == 1)) {
    blink_phase_ = 2;
    for (uint8_t i = 0; i < NUMPIXELS; i++)
      setPixelColor(i, led_off_[i]);
    show();
  } else if (millis() - blink_prev_millis_ >= interval * 2) {
    blink_phase_ = 0;
    blink_prev_millis_ = millis();
  }
}

// --- Interruptible delay ---

bool StatusLeds::interruptibleDelay(unsigned long wait) {
  unsigned long start = millis();
  while (millis() - start < wait) {
    if (input_->getBluetoothData() || input_->getKeyValue()) {
      return true;
    }
    if (state_->motion_mode == STOP &&
        car_->balance_angle_min <= car_->kalmanfilter_angle &&
        car_->kalmanfilter_angle <= car_->balance_angle_max) {
      state_->motion_mode = STANDBY;
      lightOff();
      state_->function_mode = IDLE;
      return true;
    }
  }
  return false;
}

// --- Color wheel helper ---

uint32_t StatusLeds::wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return Color(255 - pos * 3, 0, pos * 3);
  }
  if (pos < 170) {
    pos -= 85;
    return Color(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return Color(pos * 3, 255 - pos * 3, 0);
}

// --- Blocking animations ---

bool StatusLeds::pulseWhite(uint8_t wait) {
  for (int j = 0; j < 256; j++) {
    uint8_t g = gamma8(j);
    for (uint16_t i = 0; i < numPixels(); i++) {
      setPixelColor(i, Color(0, 0, 0, g));
    }
    if (interruptibleDelay(wait))
      return true;
    show();
  }
  for (int j = 255; j >= 0; j--) {
    uint8_t g = gamma8(j);
    for (uint16_t i = 0; i < numPixels(); i++) {
      setPixelColor(i, Color(0, 0, 0, g));
    }
    if (interruptibleDelay(wait))
      return true;
    show();
  }
  return false;
}

bool StatusLeds::theaterChase(uint8_t r, uint8_t g, uint8_t b, uint8_t wait) {
  for (int j = 0; j < 200; j++) {
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < numPixels(); i = i + 3) {
        setPixelColor(i + q, Color(r, g, b));
      }
      show();
      if (interruptibleDelay(wait))
        return true;
      for (uint16_t i = 0; i < numPixels(); i = i + 3) {
        setPixelColor(i + q, 0);
      }
    }
  }
  return false;
}

bool StatusLeds::rainbow(uint8_t wait) {
  for (uint16_t k = 0; k <= 100; k++) {
    for (uint16_t j = 0; j < 256; j++) {
      for (uint16_t i = 0; i < numPixels(); i++) {
        setPixelColor(i, wheel((i + j) & 255));
      }
      show();
      if (interruptibleDelay(wait))
        return true;
    }
  }
  return false;
}

bool StatusLeds::rainbowCycle(uint8_t wait) {
  for (uint16_t j = 0; j < 256 * 100; j++) {
    for (uint16_t i = 0; i < numPixels(); i++) {
      setPixelColor(i, wheel(((i * 256 / numPixels()) + j) & 255));
    }
    show();
    if (interruptibleDelay(wait))
      return true;
  }
  return false;
}

bool StatusLeds::theaterChaseRainbow(uint8_t wait) {
  for (int k = 0; k < 30; k++) {
    for (int j = 0; j < 256; j++) {
      for (int q = 0; q < 3; q++) {
        for (uint16_t i = 0; i < numPixels(); i = i + 3) {
          setPixelColor(i + q, wheel((i + j) % 255));
        }
        show();
        if (interruptibleDelay(wait))
          return true;
        for (uint16_t i = 0; i < numPixels(); i = i + 3) {
          setPixelColor(i + q, 0);
        }
      }
    }
  }
  return false;
}

bool StatusLeds::rainbowFade2White(uint8_t wait, int rainbowLoops,
                                   int whiteLoops) {
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;
  for (int k = 0; k < rainbowLoops; k++) {
    for (int j = 0; j < 256; j++) {
      for (uint16_t i = 0; i < numPixels(); i++) {
        wheelVal = wheel(((i * 256 / numPixels()) + j) & 255);
        redVal = ((wheelVal >> 16) & 0xFF) * float(fadeVal / fadeMax);
        greenVal = ((wheelVal >> 8) & 0xFF) * float(fadeVal / fadeMax);
        blueVal = (wheelVal & 0xFF) * float(fadeVal / fadeMax);
        setPixelColor(i, Color(redVal, greenVal, blueVal));
      }
      if (k == 0 && fadeVal < fadeMax - 1) {
        fadeVal++;
      } else if (k == rainbowLoops - 1 && j > 255 - fadeMax) {
        fadeVal--;
      }
      show();
      if (interruptibleDelay(wait))
        return true;
    }
  }

  if (interruptibleDelay(500))
    return true;

  for (int k = 0; k < whiteLoops; k++) {
    for (int j = 0; j < 256; j++) {
      for (uint16_t i = 0; i < numPixels(); i++) {
        setPixelColor(i, Color(0, 0, 0, gamma8(j)));
      }
      show();
    }
    if (interruptibleDelay(2000))
      return true;
    for (int j = 255; j >= 0; j--) {
      for (uint16_t i = 0; i < numPixels(); i++) {
        setPixelColor(i, Color(0, 0, 0, gamma8(j)));
      }
      show();
    }
  }

  if (interruptibleDelay(500))
    return true;
  return false;
}

bool StatusLeds::whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed,
                                  uint8_t whiteLength) {
  if (whiteLength >= numPixels())
    whiteLength = numPixels() - 1;

  unsigned int head = whiteLength - 1;
  unsigned int tail = 0;

  int loops = 100;
  int loopNum = 0;

  while (true) {
    for (int j = 0; j < 256; j++) {
      for (uint16_t i = 0; i < numPixels(); i++) {
        if ((i >= tail && i <= head) || (tail > head && i >= tail) ||
            (tail > head && i <= head)) {
          setPixelColor(i, Color(0, 0, 0, 255));
        } else {
          setPixelColor(i, wheel(((i * 256 / numPixels()) + j) & 255));
        }
      }

      if (millis() - white_rainbow_last_time_ > whiteSpeed) {
        head++;
        tail++;
        if (head == numPixels()) {
          loopNum++;
        }
        white_rainbow_last_time_ = millis();
      }

      if (loopNum == loops)
        return false;

      head %= numPixels();
      tail %= numPixels();
      show();
      if (interruptibleDelay(wait))
        return true;
    }
  }
  return false;
}
