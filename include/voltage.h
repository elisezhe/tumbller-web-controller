#ifndef VOLTAGE_H
#define VOLTAGE_H

#include <Arduino.h>

extern int low_voltage_flag;
extern unsigned long vol_measure_time;

void voltageInit();
void voltageMeasure();

#endif
