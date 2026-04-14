#include "encoders.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
// Attention : Ce header est généré automatiquement par CMake à partir de ton fichier .pio
#include "quadrature_encoder.pio.h" 

// --- CONFIGURATION DES BROCHES (Encodeurs A et B) ---
#define PIN_ENCODER_L_A 6 // (La broche B doit être la 7 obligatoirement avec le PIO)
#define PIN_ENCODER_R_A 8 // (La broche B doit être la 9 obligatoirement)

// Variables pour stocker les "offsets" logiciels lors d'un reset
int32_t offset_left = 0;
int32_t offset_right = 0;

// Sauvegarde des State Machines utilisées
uint sm_left;
uint sm_right;
PIO pio = pio0; // On utilise le bloc PIO n°0

void encoders_init(void) {
    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    
    sm_left = pio_claim_unused_sm(pio, true);
    sm_right = pio_claim_unused_sm(pio, true);

    quadrature_encoder_program_init(pio, sm_left, offset, PIN_ENCODER_L_A, 0);
    quadrature_encoder_program_init(pio, sm_right, offset, PIN_ENCODER_R_A, 0);
}

int32_t encoders_get_ticks_left(void) {
    // Lecture directe depuis la machine à états matérielle
    int32_t count = quadrature_encoder_get_count(pio, sm_left);
    // On inverse le signe si le moteur compte à l'envers en avançant
    return count - offset_left; 
}

int32_t encoders_get_ticks_right(void) {
    int32_t count = quadrature_encoder_get_count(pio, sm_right);
    return count - offset_right;
}

void encoders_reset(void) {
    // Plutôt que de réinitialiser le matériel (complexe), on enregistre l'offset actuel
    offset_left = quadrature_encoder_get_count(pio, sm_left);
    offset_right = quadrature_encoder_get_count(pio, sm_right);
}