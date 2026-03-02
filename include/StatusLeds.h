#ifndef STATUSLEDS_H
#define STATUSLEDS_H

#include "Adafruit_NeoPixel.h"
#include "Pins.h"

// Callback that returns true when an animation should abort early.
typedef bool (*AbortCheckFn)();

// Pixel layout (viewed from behind the car):
//   pixel 2 (front-left)   pixel 3 (front-right)
//   pixel 1 (back-left)    pixel 0 (back-right)
enum LedDirection : uint8_t {
  DIR_ALL,
  DIR_FRONT,
  DIR_BACK,
  DIR_LEFT,
  DIR_RIGHT,
};

class StatusLeds : public Adafruit_NeoPixel {
 public:
  StatusLeds();
  void init(AbortCheckFn abort_check = nullptr);

  // Indicator patterns (non-blocking, used with blink())
  void lightOff();
  void solid(uint32_t color);
  void flash(uint32_t color, LedDirection dir);
  void blink(uint32_t interval);

  // Blocking animations (return true if interrupted by input)
  bool pulseWhite(uint8_t wait);
  bool theaterChase(uint8_t r, uint8_t g, uint8_t b, uint8_t wait);
  bool rainbow(uint8_t wait);
  bool rainbowCycle(uint8_t wait);
  bool theaterChaseRainbow(uint8_t wait);
  bool rainbowFade2White(uint8_t wait, int16_t rainbowLoops, int16_t whiteLoops);
  bool whiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength);

  int16_t brightness;
  int8_t animation_index;

 private:
  uint32_t led_on_[NUMPIXELS];
  uint32_t led_off_[NUMPIXELS];
  AbortCheckFn abort_check_;
  uint8_t blink_phase_;
  uint32_t blink_prev_millis_;

  bool interruptibleDelay(uint32_t wait);
  uint32_t wheel(uint8_t pos);
};

#endif
