#include "Codeur.h"
#include "quadrature_encoder.pio.h"
#include <stdio.h>

PIO pio = pio0;

const uint PIN_AB_LEFT  = 2;
const uint PIN_AB_RIGHT = 4;

uint sm_left  = 0;
uint sm_right = 1;

volatile int32_t encoder_left_pos  = 0;
volatile int32_t encoder_right_pos = 0;

void encoders_init(void) {
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm_left,  PIN_AB_LEFT,  0);
    quadrature_encoder_program_init(pio, sm_right, PIN_AB_RIGHT, 0);
}

bool encoder_update_callback(struct repeating_timer *t) {
    encoder_left_pos  = quadrature_encoder_get_count(pio, sm_left);
    encoder_right_pos = quadrature_encoder_get_count(pio, sm_right);
    return true;
}

bool display_callback(struct repeating_timer *t) {
    printf("Left = %6ld  Right = %6ld\n",
           encoder_left_pos,
           encoder_right_pos);
    return true;
}
