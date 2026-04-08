#ifndef MOTEURS_H
#define MOTEURS_H

#include "hardware/pwm.h"
#include <stdint.h>

void moteurs_init(void);
void moteur_gauche_set(int vitesse);
void moteur_droit_set(int vitesse);
void moteurs_set(int vitesse_gauche, int vitesse_droite);
void moteurs_stop(void);

#endif