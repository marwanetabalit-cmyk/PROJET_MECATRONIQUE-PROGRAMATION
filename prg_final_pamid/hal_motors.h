#ifndef HAL_MOTORS_H
#define HAL_MOTORS_H

#include <stdint.h>

// Configuration des pins (ajuster selon ton câblage)
#define MOTOR_A_IN1 14   // Moteur gauche
#define MOTOR_A_IN2 15
#define MOTOR_B_IN1 16   // Moteur droit
#define MOTOR_B_IN2 17

// Initialisation des moteurs
void motors_init();

// Contrôle vitesse (-100 à +100)
void motor_left_set(int speed);    // Moteur gauche
void motor_right_set(int speed);   // Moteur droit

// Commandes prédéfinies
void motors_forward(uint8_t speed);   // Avance
void motors_backward(uint8_t speed);  // Recule
void motors_rotate_right(uint8_t speed); // Rotation droite
void motors_rotate_left(uint8_t speed);  // Rotation gauche
void motors_stop();                   // Arrêt complet

#endif