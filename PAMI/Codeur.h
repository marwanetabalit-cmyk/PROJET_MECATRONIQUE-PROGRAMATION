#ifndef CODEUR_H
#define CODEUR_H

#include "pico/stdlib.h"

void encoders_init(void);
bool encoder_update_callback(struct repeating_timer *t);
bool display_callback(struct repeating_timer *t);

#endif
