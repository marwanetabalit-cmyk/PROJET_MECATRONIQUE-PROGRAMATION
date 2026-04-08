#ifndef HAL_SERVO_H
#define HAL_SERVO_H


#include <stdint.h>

#define SERVO_PIN 14  // GPIO pour le servomoteur

// Initialisation du servomoteur
void servo_init(void);

// Définit l'angle du servomoteur (0-180 degrés)
void servo_set_angle(int angle);

// Animation de célébration
void servo_celebrate(void);

#endif