#ifndef MOTORS_H
#define MOTORS_H

#include "hardware/pwm.h"

// Déclaration des slices PWM pour les moteurs A et B
extern uint slice_A;
extern uint slice_B;

// Initialise les moteurs (configure PWM et GPIO)
void motor_init(void);

// Définit la vitesse d’un moteur (-100 à 100)
void motor_set_speed(uint slice, int speed);

#endif
