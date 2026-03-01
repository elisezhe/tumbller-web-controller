#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>

extern char key_value;
extern char key_flag;
extern char key_mode;
extern char prev_key_mode;
extern unsigned long key_mode_time;

void keyValue();
bool getKeyValue();
void keyInit();
bool getBluetoothData();

#endif
