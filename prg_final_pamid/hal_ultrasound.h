#ifndef HAL_ULTRASOUND_H
#define HAL_ULTRASOUND_H

// Configuration des pins
#define TRIG_PIN 12
#define ECHO_PIN 13

// Initialisation
void ultrasound_init();

// Mesure distance en cm (retourne -1 si erreur)
float ultrasound_measure();

// Vérifie si obstacle à moins de 'threshold_cm'
bool obstacle_detected(float threshold_cm);

#endif