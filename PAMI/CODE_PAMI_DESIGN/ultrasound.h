#ifndef ULTRASON_H
#define ULTRASON_H

#include "pico/stdlib.h"

// Initialise le capteur ultrason
void ultrason_init(void);

// Lit la distance détectée par le capteur (en cm)
// Retourne -1 si aucune distance valide
float lireDistance(void);

#endif
