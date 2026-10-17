# ELEGOO Tumbller — Self-Balancing Robot Car

Refactored firmware for the [ELEGOO Tumbller Self-Balancing Robot Car Kit](https://www.elegoo.com/products/elegoo-tumbller-self-balancing-robot-car)
([Amazon](https://www.amazon.com/dp/B0B68M74WQ)),
built with [PlatformIO](https://platformio.org/) for the Arduino Nano (ATmega328P).

> **Note:** This project has been migrated from the original Arduino IDE sketch
> to a PlatformIO project. The single-file `.ino` sketch was refactored into
> modular C++ classes with separate headers and source files, proper dependency
> management via `platformio.ini`, and a standard `src/` / `include/` / `lib/`
> layout. Build, upload, and serial monitor are all handled through PlatformIO.

## Local Web Bluetooth Controller

The `web/` directory contains a browser controller for the robot's **ELEGOO BT16**
BLE module. It connects directly through Web Bluetooth—Windows pairing and COM
ports are not used.

Requirements:

- Chrome or Edge with Web Bluetooth support
- The robot powered on and nearby
- The page served from `localhost` (Web Bluetooth requires a secure context)

Run it locally:

```bash
npm start
```

Then open `http://localhost:8080`. Click **Connect robot**, choose
**ELEGOO BT16**, and use the on-screen controls or WASD/arrow keys. Directional
buttons are hold-to-drive and automatically send the firmware's `s` stop command
when released.

Run the JavaScript tests with:

```bash
npm test
```

BLE protocol:

| Item | Value |
| --- | --- |
| Device name | `ELEGOO BT16` |
| UART service | `0000ffe0-0000-1000-8000-00805f9b34fb` |
| UART characteristic | `0000ffe1-0000-1000-8000-00805f9b34fb` |

## Hardware

| Component | Description |
| --- | --- |
| MCU | ATmega328P (Arduino Nano V3 / V4.1 CH340) |
| IMU | MPU6050 6-axis accelerometer + gyroscope (I²C) |
| Motors | 2× DC gear motors with TB6612FNG driver |
| Encoders | 2× wheel encoders (pin-change interrupts) |
| Ultrasonic | HC-SR04 distance sensor (trigger + echo) |
| IR | 2× IR receivers (left/right obstacle detection) |
| LEDs | 4× WS2812B (NeoPixel) addressable RGB LEDs |
| Bluetooth | Serial module for app control |
| Button | Mode selection button (pin 10) |
| Power | Li-ion battery with voltage monitor (A2) |

### Pin Map

| Pin | Function |
| --- | --- |
| 2 | Encoder left channel A (PCINT) |
| 3 | NeoPixel data (4 LEDs) |
| 4 | Encoder right channel A |
| 5 | Motor A PWM (left) |
| 6 | Motor B PWM (right) |
| 7 | Motor A direction (AIN1) |
| 8 | Motor driver standby (STBY) |
| 9 | IR send |
| 10 | Mode button |
| 11 | Ultrasonic trigger |
| 12 | Motor B direction (BIN1) |
| A0 | IR receiver left |
| A1 | IR receiver right |
| A2 | Battery voltage divider |
| A3 | Ultrasonic echo |

## Features

- **Self-balancing** — Kalman-filtered IMU angle fused with gyro rate,
  balanced by a PD controller at 200 Hz (5 ms timer interrupt)
- **Speed control** — PI controller with encoder feedback, updated every
  ~40 ms (8 ISR cycles), EMA-filtered
- **Turn control** — PD controller using yaw gyro rate
- **Obstacle avoidance** — HC-SR04 ultrasonic + dual IR sensors
- **Line following** — IR-based following with two sensitivity profiles
- **Stand-up recovery** — Non-blocking motor burst to flip the robot upright
  when fallen, with duration proportional to tilt angle
- **Bluetooth control** — Serial commands for directional driving, mode
  selection, and LED brightness
- **LED animations** — Solid, flash, fade, and breathing patterns on
  WS2812B LEDs with configurable brightness
- **Low-voltage protection** — Automatic cutoff when battery voltage is low

## Control Modes

| Command | Action |
| --- | --- |
| `f` / `b` / `l` / `i` | Forward / Backward / Turn left / Turn right |
| `s` | Stop (standby) |
| `1` | Follow mode (IR-based) |
| `0` | Follow mode 2 (wider range) |
| `2` | Obstacle avoidance mode |
| `3` | Cycle LED animation |
| `g` | Amber breathing glow mode |
| `4` | Stand-up maneuver |
| `5` | Startup / re-initialize balance |
| `6` | Reset LED brightness |
| `7` / `8` | Decrease / Increase LED brightness |
| `9` | LEDs off |

## Building

### Prerequisites

- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html)
  or the VS Code PlatformIO extension

### Build

```bash
platformio run
```

### Upload

Connect the Nano via USB and run:

```bash
platformio run --target upload
```

PlatformIO auto-detects the serial port. To override (e.g. when multiple
devices are connected):

```bash
platformio run --target upload --upload-port /dev/ttyUSB0
```

Or set `upload_port` in `platformio.ini`.

### Format

```bash
make format
```

Requires `clang-format` on PATH.

## Libraries

All libraries ship in `lib/` (vendored from the original ELEGOO kit and
PlatformIO registry). No additional downloads are needed.

| Library | Version | License | Source |
| --- | --- | --- | --- |
| Adafruit NeoPixel | 1.15.4 | LGPL-3.0 | [GitHub](https://github.com/adafruit/Adafruit_NeoPixel) |
| EnableInterrupt | 1.1.0 | Apache-2.0 | [GitHub](https://github.com/GreyGnome/EnableInterrupt) |
| I2Cdev | 1.0.1 | MIT | [GitHub](https://github.com/jrowberg/i2cdevlib) |
| MPU6050 | 1.0.0 | MIT | [GitHub](https://github.com/jrowberg/i2cdevlib) |
| MsTimer2 | 1.1 | LGPL-2.1 | [GitHub](https://github.com/PaulStoffregen/MsTimer2) |
| Wire | 1.0 | LGPL-2.1 | Arduino core (built-in) |

## Project Structure

```text
├── platformio.ini              PlatformIO project configuration
├── Makefile                    clang-format helper
├── include/
│   ├── BalanceCar.h            Balance ISR, PID controllers, encoder interface
│   ├── EnableInterruptWrapper.h  Multi-file compilation wrapper for EnableInterrupt
│   ├── InputHandler.h          Button + Bluetooth input abstraction
│   ├── KalmanFilter.h          Kalman filter for IMU angle estimation
│   ├── mode.h                  Motion/function mode enums and CarState
│   ├── MotorDriver.h           TB6612FNG motor driver wrapper
│   ├── PidController.h         Reusable 3-input PID controller
│   ├── Pins.h                  Hardware pin definitions
│   ├── StatusLeds.h            LED animation state machine
│   ├── Tumbller.h              Main application orchestrator
│   ├── UltrasonicSensor.h      HC-SR04 distance + obstacle detection
│   ├── VoltageMon.h            Battery voltage monitor
│   └── WheelEncoder.h          Encoder pulse accumulator with ISR
├── lib/
│   ├── Adafruit_NeoPixel/      WS2812B NeoPixel LED driver
│   ├── EnableInterrupt/        Pin-change interrupt library
│   ├── I2Cdev/                 I²C device helper library
│   ├── MPU6050/                MPU6050 IMU driver
│   └── MsTimer2/               Timer2 interrupt library
└── src/
    ├── main.cpp                Arduino setup()/loop() entry point
    ├── BalanceCar.cpp           Balance control implementation
    ├── EnableInterruptImpl.cpp  EnableInterrupt single compilation unit
    ├── InputHandler.cpp         Button + Bluetooth input implementation
    ├── KalmanFilter.cpp         Kalman filter implementation
    ├── MotorDriver.cpp          TB6612FNG motor driver implementation
    ├── PidController.cpp        PID controller implementation
    ├── StatusLeds.cpp           LED animation implementation
    ├── Tumbller.cpp             Application logic, commands, behaviors
    ├── UltrasonicSensor.cpp     Ultrasonic sensor implementation
    ├── VoltageMon.cpp           Voltage monitor implementation
    └── WheelEncoder.cpp         Wheel encoder implementation
```

## Architecture

The firmware runs a cooperative loop with a timer-driven ISR:

- **MsTimer2 ISR (5 ms)** — Reads IMU, runs Kalman filter, computes
  balance PD + speed PI + turn PD, outputs PWM to motors
- **`loop()`** — Polls input (button, Bluetooth), updates ultrasonic
  sensor, checks voltage, manages motion state transitions, runs
  autonomous behaviors, updates LED animations

Key classes:

| Class | Responsibility |
| --- | --- |
| `Tumbller` | Top-level orchestrator: input dispatch, mode management, behaviors |
| `BalanceCar` | Balance ISR, PID loops, Kalman filter, encoder reading |
| `PidController` | Generic 3-input PID with configurable dt scaling |
| `KalmanFilter` | 2-state Kalman filter (angle + gyro bias) |
| `WheelEncoder` | ISR-driven pulse counter with direction from PWM sign |
| `MotorDriver` | TB6612FNG abstraction (forward, back, turn, stop) |
| `StatusLeds` | Non-blocking NeoPixel animation state machine |
| `UltrasonicSensor` | Non-blocking HC-SR04 distance measurement |
| `InputHandler` | Button debouncing + serial/Bluetooth command parsing |
| `VoltageMon` | ADC-based battery voltage with low-voltage flag |

## Resources

- [ELEGOO Tumbller Tutorial & Downloads (V1.1)](https://www.elegoo.com/blogs/arduino-projects/elegoo-tumbller-self-balancing-robot-car-tutorial)
- [ELEGOO Tumbller V1.1 — Download Link 1](https://download.elegoo.com/?t=TumbllerV1.1)
- [ELEGOO Tumbller V1.1 — Google Drive](https://drive.google.com/file/d/1_5O1cjhr69-vvk1EDNT6Tr4CjCzjRot8/view?usp=sharing)
- [Multi-Language Manual (V1.1)](https://drive.google.com/drive/folders/1J9G9k_AA0Hsm2vCmMttLhkT9coV2mdYw?usp=sharing)
- [ELEGOO Product Page](https://www.elegoo.com/products/elegoo-tumbller-self-balancing-robot-car)

## License

This project is licensed under the [MIT License](LICENSE).

The original ELEGOO Tumbller sample code is subject to ELEGOO's own terms.
Refactoring and improvements in this repository are provided under MIT for
educational and personal use.
