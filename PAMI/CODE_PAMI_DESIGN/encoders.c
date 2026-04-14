#include "encoders.h"
#include "quadrature_encoder.pio.h"
#include <stdio.h>

// Utilisation du PIO 0 pour les codeurs
PIO pio = pio0;

// Broches des codeurs gauche et droite
const uint PIN_AB_LEFT  = 2;
const uint PIN_AB_RIGHT = 4;

// State machines utilisées pour les deux codeurs
uint sm_left  = 0;
uint sm_right = 1;

// Positions actuelles des codeurs
volatile int32_t encoder_left_pos  = 0;
volatile int32_t encoder_right_pos = 0;

// Initialisation des codeurs
void encoders_init(void) {
    pio_add_program(pio, &quadrature_encoder_program);          // Charge le programme PIO
    quadrature_encoder_program_init(pio, sm_left,  PIN_AB_LEFT,  0);  // Init codeur gauche
    quadrature_encoder_program_init(pio, sm_right, PIN_AB_RIGHT, 0);  // Init codeur droit
}

// Mise à jour périodique des positions des codeurs
bool encoder_update_callback(struct repeating_timer *t) {
    encoder_left_pos  = quadrature_encoder_get_count(pio, sm_left);
    encoder_right_pos = quadrature_encoder_get_count(pio, sm_right);
    return true;
}

// Affichage périodique des positions des codeurs
bool display_callback(struct repeating_timer *t) {
    printf("Left = %6ld  Right = %6ld\n",
           encoder_left_pos,
           encoder_right_pos);
    return true;
}
