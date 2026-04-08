#ifndef MOTORS_H
#define MOTORS_H

#include "hardware/pwm.h"

extern uint slice_A;
extern uint slice_B;

void motor_init(void);
void motor_set_speed(uint slice, int speed);

#endif
