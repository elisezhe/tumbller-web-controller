#ifndef MODE_H
#define MODE_H

enum FUNCTION_MODE {
  IDLE,
  IRREMOTE,
  OBSTACLE,
  FOLLOW,
  BLUETOOTH,
  FOLLOW2,
};

enum MOTION_MODE {
  STANDBY,
  FORWARD,
  BACKWARD,
  TURNLEFT,
  TURNRIGHT,
  STOP,
  START,
};

extern FUNCTION_MODE function_mode;
extern MOTION_MODE motion_mode;

#endif
