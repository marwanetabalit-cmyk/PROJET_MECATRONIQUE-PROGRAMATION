#ifndef CODEUR_H
#define CODEUR_H

#include "pico/stdlib.h"

// Initialise les codeurs pour mesurer la rotation des moteurs
void encoders_init(void);

// Fonction appelée périodiquement pour mettre à jour les valeurs des codeurs
bool encoder_update_callback(struct repeating_timer *t);

// Fonction appelée périodiquement pour afficher les valeurs des codeurs
bool display_callback(struct repeating_timer *t);

#endif
