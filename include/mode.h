#ifndef MODE_H
#define MODE_H

enum FUNCTION_MODE { IDLE, IRREMOTE, OBSTACLE, FOLLOW, BLUETOOTH, FOLLOW2 };
enum MOTION_MODE { STANDBY, FORWARD, BACKWARD, TURNLEFT, TURNRIGHT, STOP, START };

struct CarState {
  FUNCTION_MODE function_mode;
  MOTION_MODE motion_mode;
  CarState() : function_mode(IDLE), motion_mode(START) {
  }
};

#endif
