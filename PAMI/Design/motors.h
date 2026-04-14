#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

// Initialise les broches et les PWM pour les moteurs
void motors_init(void);

// Applique une vitesse aux moteurs (de -100 à 100)
// Des valeurs négatives font reculer le robot.
void motors_set_speed(int speed_left, int speed_right);

#endif // MOTORS_H